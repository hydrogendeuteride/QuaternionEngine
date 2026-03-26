## 현재 구현 상태 (2026-03-26)

### 완료

* 1. `stable cache`와 `live preview overlay` 분리
  * `PredictionTrackState`에 stable `cache`와 별도 `preview_overlay`를 분리했다.
  * FastPreview 결과는 stable cache를 덮지 않고 overlay에만 누적되며, full refine 시점에만 stable cache를 교체한다.
* 3. FastPreview에서 `flatten_chunk_assembly_to_cache()` 제거
  * preview chunk publish 경로에서 flat planned cache 재생성과 전체 planned render curve 재빌드를 끊었다.
  * preview draw는 chunk assembly 직접 소비 경로를 사용한다.
* 7. drag throttle을 wall-time으로 전환
* 드래그 프로파일링용 전용 ImGui 디버그 창 추가

### 부분 완료

* 2. CPU draw도 chunk assembly 직접 소비
  * planned preview draw는 GPU off 경로에서도 chunk `render_curve` / `frame_segments`를 직접 소비한다.
  * 다만 draw 전체가 chunk-first 구조로 완전히 정리된 것은 아니고, stable fallback과의 접합부가 아직 남아 있다.
* 4. pick를 preview 전용으로 분리
  * planned pick은 preview chunk assembly를 직접 소비하고, overlay가 아직 덮지 않은 prefix만 stable planned cache로 fallback한다.
  * 전용 `preview_pick_cache`나 chunk별 invalidation 체계는 아직 없다.
* 5. visual / exact / pick window 분리
  * drag 중 exact preview window를 별도 cap 해서 display horizon과 완전히 같이 커지지 않게 만들었다.
  * 하지만 visual / exact / pick 정책이 완전히 독립된 파라미터로 분리되진 않았다.
* 6. frame spec versioning / preview overlay invalidation 정리
  * display frame 변경 시 preview overlay 전체 invalidate는 넣었다.
  * 하지만 `frame_revision`/`display_frame_key` 기반의 버전 관리나 chunk 단위 incremental invalidation은 아직 없다.

### 미완료

* preview chunk build에서 `frame_samples`를 lazy/optional로 줄이는 작업
* preview 전용 pick cache 구조 정식 도입
* `PreviewPatchRequest`와 solver 정책에서 visual / exact / pick window를 완전히 분리
* frame spec 버전 정보를 overlay/chunk에 명시적으로 싣는 작업

가능합니다. 지금 코드는 이미 그 방향의 기반이 꽤 깔려 있습니다. 문제는 solver가 아니라, 그 뒤의 cache, draw, pick 경로가 다시 “큰 한 덩어리 곡선”으로 돌아가 버린다는 점입니다. 말하자면 청크 스트리밍 레일은 깔아놨는데, 마지막 역 앞에서 다시 한 줄로 합쳐 병목이 생기는 구조입니다.

핵심 진단부터 정리하면 이렇습니다.

`src/game/orbit/orbit_prediction_service.h/.cpp` 쪽은 이미 꽤 잘 되어 있습니다. `SolveQuality::FastPreview`, `PreviewPatchRequest`, `PublishStage::FastPreviewFP0/FP1`, `PublishedChunk`가 있고, 실제로 solver는 preview patch를 기준으로 FP0, FP1을 나눠 publish합니다. `build_prediction_solve_plan()`도 청크 단위 계획을 만들고, `resolve_chunk_profile_id()`는 preview 구간을 `InteractiveExact`로 올립니다. `PlannedChunkCacheEntry`까지 있어서 chunk 재사용 레일까지 이미 있습니다.

그런데 실제 체감 성능은 그다음 단계에서 깎입니다.

첫째, `gameplay_prediction_derived_service.cpp`에서 preview chunk를 잘 만들고도 바로 `flatten_chunk_assembly_to_cache()`로 다시 평탄화합니다. 그리고 `gameplay_prediction_cache_internal.h`에서 그 평탄화된 전체 planned segment로 `OrbitRenderCurve::build()`를 다시 돌립니다. 장기 플롯일수록 이 비용이 커집니다.

둘째, `gameplay_state_prediction_runtime.cpp`의 `merge_preview_planned_prefix_cache()`가 이전 prefix와 새 patch/tail을 합쳐서 또 전체 planned render curve를 다시 만듭니다. 즉, 스트리밍 결과를 받아도 매번 “전체 planned 경로 재구성” 비용을 다시 내고 있습니다.

셋째, `gameplay_state_prediction_draw.cpp`에서는 `planned_chunk_assembly`를 바로 그리는 경로가 사실상 `gpu_generate_enabled`일 때만 열려 있습니다. CPU draw 경로는 이미 chunk마다 `render_curve`가 만들어져 있는데도 여전히 flat cache 중심으로 갑니다. `OrbitChunk`에 `render_curve`와 `gpu_roots`를 다 넣어놓고도, CPU 쪽에서는 그걸 거의 못 쓰고 있습니다.

넷째, pick 경로도 아직 flat planned cache 중심입니다. `planned_pick_window`가 크면 `build_pick_lod()`나 pick segment cache rebuild가 길게 돌아가고, maneuver gizmo 같은 상호작용 체감도 같이 떨어집니다.

다섯째, 지금은 “보여줄 범위”, “즉시 다시 계산할 범위”, “픽킹할 범위”가 거의 같은 축으로 묶여 있습니다.
`refresh_prediction_preview_anchor()`에서 `patch_window_s = max(maneuver_plan_preview_window_s(), maneuver_post_node_coverage_s())`로 가고, solver 쪽 `preview_fp0_window_s()`는 그 값을 다시 `patch_window_s * 2`로 키웁니다. 즉 preview 시간이 길어질수록 FP0 자체가 커지고, draw/pick도 같이 커집니다. 긴 미리보기 창 하나가 여러 비용을 동시에 불러오는 구조입니다.

여섯째, 현재 chunk preview 경로가 `reuse_existing_base_frame`에 묶여 있어서 `Inertial`이나 `LVLH` 프레임에선 아예 청크 경로가 꺼질 수 있습니다. `frame_supports_live_base_frame_reuse()`가 그 둘을 제외하고 있고, derived service의 `use_chunk_path = reuse_existing_base_frame && !published_chunks.empty()` 조건이 그 위에 얹혀 있습니다. 그래서 어떤 표시 프레임에선 이미 만들어 둔 chunk 시스템이 사실상 안 켜질 가능성이 있습니다.

그래서 구조 계획의 핵심은 하나입니다.

**편집 중에는 planned path의 대표 표현을 flat cache가 아니라 chunk assembly로 바꿔야 합니다.**
그리고 마지막 full refine가 왔을 때만 flat cache를 갈아끼우는 방식으로 가야 합니다.

권장 구조는 이렇게 가져가면 됩니다.

### 1. `stable cache`와 `live preview overlay`를 분리

`PredictionTrackState`를 지금의 단일 `cache` 중심에서 다음처럼 나누는 게 좋습니다.

* `track.cache`: 마지막 full solve 결과, 안정 버전
* `track.preview_overlay`: live preview 중인 planned chunk들
* `track.pick_cache`: stable용
* `track.preview_pick_cache` 또는 chunk별 pick cache: live preview용

즉, 드래그 중에는 `track.cache`를 계속 덮어쓰지 않고 그대로 둡니다.
새로 들어오는 FP0/FP1 결과는 `track.preview_overlay`에만 누적합니다.

이렇게 되면 long planned curve 전체를 매번 다시 빌드하지 않고도, 사용자는 바뀐 chunk만 즉시 볼 수 있습니다.

### 2. draw는 “preview overlay 우선, stable fallback 보강”으로

이건 현재 GPU chunk 경로가 이미 반쯤 하고 있는 방식입니다. `compute_uncovered_ranges()`까지 이미 있습니다.

렌더 순서를 이렇게 바꾸면 됩니다.

```text
Full solve -> stable cache 유지
FastPreview FP0/FP1 -> preview chunk overlay 갱신
Draw:
  1) preview chunk assembly를 먼저 그림
  2) preview가 아직 안 덮은 시간 범위는 stable cache로 채움
Full refine 완료:
  stable cache를 새 결과로 swap
  preview overlay clear
```

핵심은 `gameplay_state_prediction_draw.cpp`에서 chunk draw를 GPU 전용으로 두지 않는 것입니다.

지금도 chunk마다 이미 `chunk.render_curve`가 있으니, GPU가 꺼져 있어도 각 chunk에 대해 `Draw::draw_adaptive_curve_window(..., chunk.render_curve, ...)`를 호출하면 됩니다. 이건 구조적으로 매우 저위험입니다. 데이터가 이미 있습니다.

### 3. FastPreview에서는 `flatten_chunk_assembly_to_cache()`를 끊기

가장 체감이 큰 변경입니다.

`gameplay_prediction_derived_service.cpp`에서 FastPreview + published chunks인 경우:

* chunk frame data만 만든다
* `flatten_chunk_assembly_to_cache()`는 하지 않는다
* `OrbitRenderCurve::build(cache.trajectory_segments_frame_planned)` 전체 재생성도 하지 않는다
* 필요하면 `maneuver_previews` 같은 UI용 최소 데이터만 별도 전달한다

즉, preview 결과는 “chunk delta”로만 publish합니다.
full solve 결과만 flat cache를 만든다고 생각하면 됩니다.

여기서 추가로 좋은 점이 하나 더 있습니다. `OrbitChunk.frame_samples`는 현재 사실상 flatten 호환용이라 preview 중에는 거의 필요 없습니다. draw와 pick은 segment / render curve 기반으로 충분하니, preview chunk build에선 sample 생성도 생략하거나 매우 성기게 줄일 수 있습니다.

### 4. pick도 preview 전용으로 분리

지금 구조에선 pick이 long planned cache에 계속 끌려다닙니다. maneuver gizmo 체감이 나빠지는 이유 중 하나입니다.

권장 방향은 두 가지입니다.

가벼운 1차안:

* 드래그 중에는 planned pick을 preview chunk 범위만 갱신
* 먼 tail은 stable pick cache를 그대로 쓰거나, 아예 freeze
* gizmo 주변과 anchor 주변만 live pick

더 정석인 2차안:

* chunk별 pick cache를 둔다
* visible / interactive chunk만 rebuild
* generation과 frame spec이 바뀐 chunk만 invalidate

이렇게 하면 장기 tail이 있어도 gizmo 조작감이 꺼끌거리지 않습니다.

### 5. preview window를 3개로 분리

지금 제일 아픈 결합점입니다. 하나의 preview 값이 solver, draw, pick 비용을 다 끌고 갑니다.

분리해야 할 값은 최소 이 셋입니다.

* `visual_preview_window_s`: 유저가 화면에서 보고 싶은 범위
* `interactive_exact_window_s`: 드래그 중 즉시 정밀 재계산할 범위
* `interactive_pick_window_s`: live pick을 유지할 범위

그리고 solver 쪽 `PreviewPatchRequest`도 `patch_window_s` 하나로 몰지 말고, 적어도 “즉시 정밀 구간”은 별도 cap을 두는 게 좋습니다. 지금처럼 `preview_fp0_window_s = patch_window_s * 2`면 preview를 길게 잡는 순간 FP0이 비대해집니다.

실무적으로는 이렇게 가져가는 게 좋습니다.

* 근처 몇 시간 또는 몇 궤도: exact, 즉시 갱신
* 그 뒤 긴 tail: 이전 stable tail을 임시 표시
* 선택적으로 아주 성긴 new tail ghost를 빠르게 깔아줌
* 마우스 업 또는 잠시 정지 후: full refine로 전체 확정

이러면 “긴 플롯 전체에서 바뀐 방향성”도 보이고, “바로 눈앞 patch”는 즉시 따라옵니다.

### 6. frame spec 버전 관리 추가

지금 `planned_chunk_assembly`는 frame-derived 데이터인데, frame spec 자체를 들고 있지 않습니다.
`gameplay_state_prediction_frames.cpp`에서 display frame이 바뀌면 full cache는 재생성되는데, preview chunk 쪽은 별도 invalidation이나 rebuild 체계가 약합니다.

그래서 preview overlay에 최소한 이것들은 있어야 합니다.

* `resolved_frame_spec`
* `frame_revision` 또는 `display_frame_key`
* `generation_id`

프레임이 바뀌면 preview overlay 전체를 invalidate하거나, 나중엔 chunk별 incremental rebuild로 가면 됩니다.

### 7. 드래그 throttle은 sim time 말고 wall time으로

이건 구조 보강 포인트입니다.

현재 `kDragRebuildMinIntervalS` 체크가 `now_s - track.cache.build_time_s`로 가는데, `build_time_s`는 solver에서 `request.sim_time_s`로 저장됩니다. 즉, 시뮬레이션 시간이 멈춘 상태에서 maneuver 편집을 하면 cadence가 이상해질 수 있습니다.

preview request throttle은 `std::chrono::steady_clock` 기반 wall time으로 분리하는 게 맞습니다.
solver horizon은 sim time, UI cadence는 wall time, 이렇게 역할을 나누는 편이 안전합니다.

---

## 파일별로 어디를 손대면 되나

가장 중요한 포인트만 찍으면 이렇습니다.

`src/game/states/gameplay/prediction/gameplay_state_prediction_types.h`

* `PredictionTrackState`에 `preview_overlay` 추가
* `OrbitChunk`에 optional flags나 frame spec revision 추가
* preview용 pick cache 분리

`src/game/states/gameplay/prediction/gameplay_prediction_derived_service.cpp`

* FastPreview에서 `flatten_chunk_assembly_to_cache()` 제거
* `out.valid` 기준을 cache뿐 아니라 chunk assembly까지 확장
* preview는 delta chunk publish 중심으로 변경
* preview chunk build 시 sample 생략 옵션 추가

`src/game/states/gameplay/prediction/gameplay_prediction_cache_internal.h`

* `build_chunk_frame_data()`를 preview/full 용도로 분기
* `frame_samples` lazy 또는 optional
* flatten은 full solve 전용 또는 호환 모드 전용으로 축소

`src/game/states/gameplay/prediction/gameplay_state_prediction_runtime.cpp`

* `merge_preview_planned_prefix_cache()`를 preview의 정본 경로에서 제거
* `track.cache`는 stable 유지
* `track.planned_chunk_assembly` 또는 새 preview overlay만 갱신
* preview 완료 후 full refine 시점에만 stable cache swap
* request cadence를 wall time 기반으로 분리

`src/game/states/gameplay/prediction/gameplay_state_prediction_draw.cpp`

* chunk assembly draw를 GPU on일 때만 쓰지 말고 CPU path에도 확장
* chunk별 `render_curve` 직접 draw
* fallback range는 stable cache draw
* `quality_state`로 preview patch와 final tail 시각 차등 가능

`src/game/states/gameplay/prediction/gameplay_state_prediction_frames.cpp`

* frame 변경 시 preview overlay invalidation 또는 incremental rebuild

`src/game/orbit/prediction/orbit_prediction_service_internal.h`
`src/game/orbit/prediction/orbit_prediction_service_policy.cpp`

* `preview_fp0_window_s()`와 chunk profile 정책을 visual window와 분리
* 즉시 정밀 구간 cap 추가
* 필요하면 coarse tail preview stage 추가

`src/core/orbit_plot/orbit_plot.h`
`src/render/passes/orbit_plot.cpp`

* 여기는 큰 구조 변경보다, preview overlay를 GPU root batch로 쓸 때의 소비 경로만 유지하면 됩니다. 기반은 이미 충분합니다.

---

## 구현 우선순위

가장 효과 대비 리스크가 좋은 순서로 적으면 이렇습니다.

1. **Preview를 stable cache 위 overlay로 바꾸기**
   FastPreview에서 flat merge를 멈추고, draw를 chunk-first로 바꿉니다. 여기서 체감 성능이 가장 많이 오릅니다.

2. **CPU draw도 chunk assembly 직접 소비하게 만들기**
   현재 GPU 설정에 따라 체감이 갈리는데, 이걸 없애야 합니다.

3. **pick를 preview 전용으로 분리**
   gizmo 조작감이 같이 좋아집니다.

4. **visual / exact / pick window 분리**
   긴 preview에서도 FP0가 비대해지지 않게 합니다.

5. **wall-time throttle + frame versioning**
   남는 거친 모서리를 정리합니다.

---

## 결론

정리하면, **장기 궤도 플롯에서도 실시간 수정 사항을 눈에 보이게 하는 건 충분히 가능합니다.**
게다가 이 프로젝트는 그걸 위해 필요한 solver 쪽 청크화, staged publish, chunk cache까지 이미 갖고 있습니다.

지금 부족한 건 “새 solver”가 아니라, **preview를 flat cache로 되돌리지 않는 구조 전환**입니다.

가장 추천하는 방향은 이것입니다.

* 편집 중 planned path의 정본은 `flat planned cache`가 아니라 `preview chunk overlay`
* stable full cache는 그대로 유지
* renderer는 overlay를 먼저 그리고, 비어 있는 시간대만 stable tail로 채움
* full refine가 끝나면 그때만 stable cache를 교체

이렇게 바꾸면 long preview에서도 maneuver gizmo 드래그가 훨씬 즉답형으로 바뀔 가능성이 큽니다. 다음 단계는 이 구조를 기준으로 함수 단위 수정 순서를 실제 코드 패치 체크리스트로 내리는 것입니다.
