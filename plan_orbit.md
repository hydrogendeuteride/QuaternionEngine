Phase 2: FastPreview + preview_patch 구현 계획

 Context

 궤도 예측 시스템에서 기동 노드 드래그 시 SolveQuality::Full만 사용하므로 7일+ horizon에서 반응이 느리다.

 핵심 아이디어: 드래그 중에는 선택 노드 주변 local patch만 Exact 정밀도로 빠르게 풀고, 나머지는 거칠게 또는 나중에
 처리. solver는 preview patch를 먼저 publish하고 tail은 이후에 publish (staged publish).

 ---
 Step 1: 서비스 레이어 타입 추가

 파일: src/game/orbit/orbit_prediction_service.h

 1. SolveQuality enum에 FastPreview = 1 추가
 2. PredictionChunkBoundaryFlags에 추가:
   - PreviewAnchor = 1u << 5u
   - PreviewChunk = 1u << 6u
 3. 새 enum 추가:
 enum class ChunkQualityState : uint8_t { Final = 0, PreviewPatch };
 enum class PublishStage : uint8_t { PreviewStreaming = 0, PreviewFinalizing };
 4. PublishedChunk struct 추가:
 struct PublishedChunk {
     uint32_t chunk_id{0};
     ChunkQualityState quality_state{ChunkQualityState::Final};
     double t0_s, t1_s;
     bool includes_planned_path{false};
     bool reused_from_cache{false};
 };
 5. Request에 PreviewPatchSpec sub-struct + 필드 추가:
 struct PreviewPatchSpec {
     bool active{false};
     bool anchor_state_valid{false};
     double anchor_time_s{NaN};
     double visual_window_s{0.0};
     double exact_window_s{0.0};
     orbitsim::State anchor_state_inertial{};
 };
 PreviewPatchSpec preview_patch{};
 6. Result에 추가:
   - PublishStage publish_stage{PublishStage::PreviewFinalizing};
   - std::vector<PublishedChunk> published_chunks{};

 ---
 Step 2: Gameplay 타입 추가

 파일: src/game/states/gameplay/prediction/gameplay_state_prediction_types.h

 1. PredictionPreviewRuntimeState enum 추가:
 enum class PredictionPreviewRuntimeState : uint8_t {
     Idle = 0, EnterDrag, DragPreviewPending, PreviewStreaming, AwaitFullRefine
 };
 2. PredictionPreviewAnchor struct 추가:
 struct PredictionPreviewAnchor {
     bool valid{false};
     int anchor_node_id{-1};
     double anchor_time_s{NaN};
     double request_window_s{0.0};
     double visual_window_s{0.0};
     double exact_window_s{0.0};
 };
 3. PredictionTrackState에 필드 추가:
   - PredictionPreviewRuntimeState preview_state{Idle};
   - PredictionPreviewAnchor preview_anchor{};
   - double preview_entered_at_s{NaN};
   - double preview_last_anchor_refresh_at_s{NaN};
   - double preview_last_request_at_s{NaN};
   - clear_runtime()에 이 필드들 초기화 추가

 파일: src/game/states/gameplay/gameplay_settings.h

 4. ManeuverPlanWindowSettings struct 추가:
 struct ManeuverPlanWindowSettings {
     double preview_window_s{180.0};
     double solve_margin_s{300.0};
 };

 파일: src/game/states/gameplay/gameplay_state.h

 5. 멤버 추가:
   - ManeuverPlanWindowSettings _maneuver_plan_windows{};
   - bool _maneuver_plan_live_preview_active{false};
 6. 메서드 선언 추가:
   - void refresh_prediction_preview_anchor(PredictionTrackState &track, double now_s, bool with_maneuvers);
   - double prediction_display_window_s(PredictionSubjectKey key, double now_s, bool with_maneuvers) const;
   - double prediction_preview_exact_window_s(const PredictionTrackState &track, double now_s, bool with_maneuvers)
 const;

 ---
 Step 3: Planner — preview boundary 삽입

 파일: src/game/orbit/prediction/orbit_prediction_service_policy_chunking.cpp

 build_prediction_solve_plan()에서 maneuver boundary 삽입 후:

 1. request.preview_patch.active이면:
   - PreviewAnchor boundary at anchor_time_s
   - boundary at anchor_time_s + exact_window_s
   - boundary at anchor_time_s + 2 * exact_window_s
 2. resolve_chunk_profile_id() 수정: preview patch 범위 [anchor_time_s, anchor_time_s + 2*exact_window_s] 내 chunk는
 PredictionProfileId::Exact 반환
 3. chunk 생성 시 preview 범위 chunk에 PreviewChunk flag 추가, anchor 시작 chunk에 PreviewAnchor flag 추가

 기존 로직에서 allow_reuse = (profile_id != Exact)이므로 Exact preview chunk는 자동으로 allow_reuse = false

 검증 테스트: InsertsManeuverAndPreviewBoundaries, PreviewAndManeuverChunksTightenProfiles

 ---
 Step 4: Preview Anchor 시스템 구현

 파일: 새 파일 또는 기존 gameplay_state_prediction_runtime_requests.cpp에 추가

 refresh_prediction_preview_anchor() 구현:
 - 드래그 활성 + _maneuver_plan_live_preview_active 체크
 - _maneuver_state.selected_node_id로 선택 노드 찾기
 - preview_anchor 채우기: anchor_node_id, anchor_time_s = node.time_s, exact_window_s = solve_margin_s,
 request_window_s = prediction_display_window_s()
 - 상태 전이: Idle → EnterDrag (드래그 시작), PreviewStreaming → AwaitFullRefine (드래그 종료)
 - preview_last_anchor_refresh_at_s = now_s

 prediction_display_window_s() 구현:
 - live preview 비활성: 기존 prediction_required_window_s() 위임
 - live preview 활성: 선택 노드까지의 offset + solve_margin_s를 포함한 전체 display horizon 반환

 prediction_preview_exact_window_s() 구현:
 - preview anchor 유효하면 anchor.exact_window_s (= solve_margin_s)
 - 아니면 solve_margin_s fallback

 검증 테스트: PreviewAnchorCacheSeparatesPatchWindowFromDisplayWindow,
 PreviewAnchorCacheExtendsVisualWindowBeyondExactWindowDuringDrag,
 PreviewAnchorCacheTransitionsToAwaitFullRefineAfterDragEnds

 ---
 Step 5: Request 경로 변경

 파일: src/game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_requests.cpp

 request_orbiter_prediction_async() 수정:

 1. 요청 빌드 전에 refresh_prediction_preview_anchor(track, now_s, with_maneuvers) 호출
 2. track.preview_anchor.valid && drag active && _maneuver_plan_live_preview_active이면:
   - solve_quality = SolveQuality::FastPreview
   - request.preview_patch 채우기:
       - active = true, anchor_time_s, visual_window_s, exact_window_s
     - anchor_state_valid = true (기존 cache에서 anchor_time_s의 state 샘플링 가능 시)
     - anchor_state_inertial: track.cache.trajectory_inertial에서 anchor_time_s 보간
 3. 모든 maneuver impulse 유지 (upstream + downstream)
 4. 제출 후: track.preview_state = DragPreviewPending, track.preview_last_request_at_s = now_s

 검증 테스트: RequestOrbiterPredictionTracksPreviewRequestTimestamp,
 FastPreviewRequestKeepsUpstreamManeuversBeforeAnchor, FastPreviewRequestKeepsDownstreamManeuversWithinDisplayHorizon

 ---
 Step 6: Compute — staged publish

 파일: src/game/orbit/prediction/orbit_prediction_service_compute.cpp

 compute_prediction() 수정:

 1. FastPreview + preview_patch.active일 때:
   - solve plan의 chunk를 preview range (Exact/PreviewChunk flag)와 tail range로 분리
   - Preview range 먼저 solve → 각 chunk 완료 시 intermediate Result publish:
       - publish_stage = PreviewStreaming
     - published_chunks에 ChunkQualityState::PreviewPatch
     - preview 범위의 trajectory_segments_inertial_planned만 포함
   - Tail range solve → Result publish:
       - publish_stage = PreviewFinalizing
     - published_chunks에 ChunkQualityState::Final
 2. 일반 Full 요청: 기존 단일 publish 유지하되 published_chunks 채우기 (Final quality)

 주의: 같은 generation_id로 여러 result publish — poll_completed()가 여러 번 호출됨. should_continue_job() cancellation
  체크가 staged publish 사이에 job을 취소하지 않도록 확인

 검증 테스트: PredictionServicePublishesPreviewPatchAndTailAsSeparateChunkStages

 ---
 Step 7: Result 소비 변경

 파일: src/game/states/gameplay/prediction/runtime/gameplay_state_prediction_runtime_solver.cpp

 apply_completed_prediction_result() 수정:

 1. result.publish_stage == PreviewStreaming:
   - planned trajectory 데이터를 cache에 부분 적용 (preview patch 영역만)
   - maneuver_previews 업데이트
   - derived request 큐잉 (frame transform)
   - track.preview_state = PreviewStreaming
 2. result.publish_stage == PreviewFinalizing:
   - 기존 전체 적용 경로
   - track.preview_state = Idle (또는 AwaitFullRefine에서 Full 결과 대기 중이면 적절히 전이)

 ---
 Step 8: Derived/Draw 최적화

 파일: src/game/states/gameplay/prediction/gameplay_prediction_derived_service.cpp

 1. derived request 빌드 시 solve_quality == FastPreview && publish_stage == PreviewStreaming이면:
   - build_planned_render_curve = false
   - metrics 계산 skip (optional, 추가 최적화)

 파일: src/game/states/gameplay/prediction/draw/gameplay_state_prediction_draw_prepare.cpp

 2. draw prepare에서 track.preview_state == PreviewStreaming이면:
   - render curve 대신 raw segments 사용 (기존 fallback 경로 활용)
   - 이미 draw_track.cpp에서 render_curve_frame_planned.empty() 체크로 raw segment fallback 있음

 ---
 구현 순서 및 의존성

 Step 1 (service.h 타입) ──┬──→ Step 3 (planner)
                           │
 Step 2 (gameplay 타입) ───┤
                           │
                           └──→ Step 4 (anchor) → Step 5 (request) → Step 6 (compute) → Step 7 (result) → Step 8 (draw)

 - Step 1~2: 타입만 추가, 병렬 가능
 - Step 3: planner 테스트 통과 (Step 1 의존)
 - Step 4~5: gameplay 통합 (Step 1+2 의존)
 - Step 6~8: solver → result → draw 순차

 검증 방법

 1. 각 Step 완료 후 cmake --build build --target vulkan_engine_tests -- -j$(nproc) 빌드
 2. ctest --test-dir build -R "OrbitPredictionPlanner" — Step 3 후
 3. ctest --test-dir build -R "GameplayPredictionManeuver" — Step 5 후
 4. ctest --test-dir build -R "PredictionCacheInternal" — Step 6 후
 5. 전체: ctest --test-dir build — Step 8 후
 6. 실행 후 maneuver 드래그 시 궤도 프리뷰 반응 속도 체감 확인
