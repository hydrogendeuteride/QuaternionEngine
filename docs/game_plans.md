# Space Combat Game — Development Plan

이 문서는 현재 엔진/게임 상태에서 간단한 전투 데모까지 가기 위한 로드맵이다.
각 단계는 이전 단계 위에 쌓이며, 단계별로 동작하는 빌드를 유지한다.

## 현재 상태 요약

### 있는 것
- **Entity-Component**: `Entity`, `IComponent`, `Component<T>` CRTP, `EntityManager`, `GameWorld` + `EntityBuilder`
- **ShipController**: 키보드 추력/토크, SAS (stability assist)
- **궤도 시뮬레이션**: N-body sim, 궤도 예측, 타임 워프, 레일/물리 전환, 기동 노드
- **Jolt 물리**: 리지드바디, 충돌 콜백, raycast, sweep, overlap
- **충돌 레이어**: Player(4), Enemy(5), Projectile(6), Trigger(7), Debris(8) — 정의는 되어 있으나 Projectile/Enemy/Debris는 미사용
- **충돌 콜백**: `ContactLogEntry`로 로깅만 함, 데미지 전달 없음
- **시나리오 시스템**: `ScenarioConfig` (천체 + 오비터 선언적 정의), JSON 로더
- **렌더**: 파티클, 로켓 플룸, 디버그 드로우, 오비트 플롯
- **오디오**: miniaudio 시스템
- **부동 소수점 원점**: 위치/속도 리베이싱

### 없는 것 (전투 데모에 필요)
- 런타임 엔티티 스폰 (발사체 등 — 현재 모든 엔티티는 시나리오 초기화 시 생성)
- 무기/발사체 시스템
- 데미지/HP 시스템
- 팩션/태깅 (아군 vs 적군 구분)
- 타겟팅 시스템
- 전투 HUD (타겟 정보, 리드 인디케이터, HP 바)
- AI (적 기동/사격)
- 파괴 이펙트

---

## Phase 0: 씬 시스템 다듬기

전투 시스템을 올리기 전에 기존 씬/엔티티 인프라를 정리한다.

### 0-1. 런타임 엔티티 스폰 경로 정리
현재 `GameWorld::EntityBuilder`는 시나리오 초기화 시에만 사용된다.
게임 플레이 중 발사체/파편을 동적으로 스폰하려면:

- [ ] `GameplayState`에서 `_world.builder(name).transform(t).render_primitive(...).physics(...).build()` 패턴으로
      임의 시점에 엔티티를 스폰할 수 있는지 확인 → 안 되면 경로 수정
- [ ] 스폰된 엔티티의 물리 바디 생성이 `_physics_context->origin_world()`와 올바르게 동기화되는지 확인
- [ ] 스폰된 엔티티가 `EntityManager::sync_to_render()`에 자동 반영되는지 확인
- [ ] 동적 스폰된 엔티티의 파괴 경로: `GameWorld::destroy_entity()` → 물리 바디 + 렌더 인스턴스 정리 확인

### 0-2. 엔티티 태깅 / 팩션
아군과 적군을 구분할 최소한의 시스템:

- [ ] `Entity`에 `uint32_t faction` 필드 추가 (0=Neutral, 1=Player, 2=Enemy)
      또는 `OrbiterInfo`에 faction 필드 추가 — 어느 쪽이 맞는지는 설계 판단
- [ ] `ScenarioConfig::OrbiterDef`에 faction 필드 추가 → 시나리오에서 적 함선 배치 가능
- [ ] 충돌 레이어 매트릭스 설정: Player↔Enemy 충돌 O, Player↔Player 충돌 X, Projectile↔Projectile 충돌 X 등

### 0-3. 충돌 콜백 → 컴포넌트 디스패치
현재 충돌 콜백은 `GameplayState::sync_player_collision_callbacks()`에서 람다로 직접 처리한다.
이걸 컴포넌트 기반으로 바꿔야 발사체 히트 → 데미지 전달이 자연스럽다:

- [ ] `IComponent`에 `on_collision(ComponentContext&, const CollisionEvent&)` 가상 함수 추가 (기본 no-op)
- [ ] 충돌 콜백에서 `user_data`(= entity ID) → 엔티티 조회 → 모든 컴포넌트에 `on_collision` 디스패치
- [ ] 기존 `ContactLogEntry` 로깅은 유지하되, 컴포넌트 디스패치와 병행

---

## Phase 1: 발사체 시스템 (Projectile)

가장 먼저 "쏘면 날아가고 맞으면 뭔가 일어나는" 기본 루프를 만든다.

### 1-1. ProjectileComponent
```
src/game/component/projectile.h
src/game/component/projectile.cpp
```

- 소유자 엔티티 ID (발사한 함선)
- 수명 타이머 (TTL, 기본 ~5초)
- 데미지 값
- `on_fixed_update`: TTL 감소 → 0 이하면 자기 엔티티 파괴 요청
- `on_collision`: 상대 엔티티에 데미지 전달 → 자기 엔티티 파괴 요청

### 1-2. 발사체 스폰 함수
`GameplayState` 또는 별도 유틸에:

```cpp
EntityId spawn_projectile(const WorldVec3& muzzle_pos_world,
                          const glm::vec3& muzzle_velocity,  // 함선 속도 + 탄속 방향
                          float damage,
                          EntityId owner);
```

- `_world.builder("projectile_N")`으로 생성
- Sphere(0.1m) 물리 바디, Layer::Projectile, mass ~1kg
- 렌더: 작은 primitive (Sphere) 또는 나중에 트레이서 이펙트
- 초기 속도 = `muzzle_velocity`
- `ProjectileComponent` 부착

### 1-3. ShipController에 발사 키 바인딩
- `Space` 또는 마우스 좌클릭 → 발사
- 쿨다운 (예: 0.2초)
- 발사 시: 함선 forward 방향 × 탄속 + 함선 현재 속도 → `spawn_projectile()`

### 1-4. 충돌 레이어 설정
```cpp
physics->set_layer_collision(Layer::Projectile, Layer::Player, false);    // 자기 발사체에 안 맞음
physics->set_layer_collision(Layer::Projectile, Layer::Projectile, false); // 탄끼리 충돌 X
physics->set_layer_collision(Layer::Projectile, Layer::Dynamic, true);     // 적/NPC 히트
physics->set_layer_collision(Layer::Projectile, Layer::Enemy, true);       // 적 히트
```
참고: 자기 발사체 회피는 owner ID 체크로 처리 (또는 스폰 후 짧은 무적 시간)

### 1-5. 발사체 풀링 / 정리
- 동시 활성 발사체 상한 (예: 64개) — 초과 시 가장 오래된 것 제거
- `on_fixed_update`에서 TTL 만료 체크
- 파괴 요청은 즉시 처리하지 않고 프레임 끝에 일괄 처리 (재진입 방지)

---

## Phase 2: 데미지 / HP 시스템

### 2-1. HealthComponent
```
src/game/component/health.h
src/game/component/health.cpp
```

- `float max_hp`, `float current_hp`
- `void apply_damage(float amount, EntityId source)`
- `on_collision` 오버라이드: 상대가 `ProjectileComponent`를 가지고 있으면 데미지 적용
- HP ≤ 0 → 파괴 요청 플래그

### 2-2. 파괴 처리
- 엔티티 파괴 시 파티클 이펙트 스폰 (이미 파티클 패스 있음)
- 옵션: 파편(Debris) 엔티티 몇 개 스폰 (Layer::Debris, 짧은 TTL)
- `OrbiterInfo`에서도 제거 → 궤도 예측 정리

### 2-3. 기본 시나리오에 적 함선 추가
`default_earth_moon_config()`에 또는 새 시나리오에:
- faction=Enemy인 오비터 1~2개 추가 (근접 궤도, 약간의 상대 속도)
- `HealthComponent` 부착
- 플레이어에게도 `HealthComponent` 부착

---

## Phase 3: 타겟팅 + 전투 HUD

### 3-1. 타겟팅 시스템
별도 컴포넌트 또는 GameplayState 멤버:

- 현재 선택된 타겟 (`EntityId`)
- 타겟 순환 키 (Tab)
- 마우스 클릭 타겟 선택 (BVH 피킹 이미 있음)
- 타겟 유효성 검증 (파괴/범위 초과 시 자동 해제)

### 3-2. 전투 HUD (ImGui 오버레이)
- 타겟 정보: 이름, 거리, 상대 속도, HP 바
- 리드 인디케이터 (선도각): `target_pos + target_vel * (distance / projectile_speed)` → 스크린 좌표로 투영
- 자함 HP 바
- 무기 쿨다운 표시
- 탄약 (무한 or 유한 — 나중에 결정)

### 3-3. 타겟 3D 마커
- 디버그 드로우 또는 별도 렌더로 타겟 주변 브래킷 표시
- 화면 밖일 때 가장자리 화살표

---

## Phase 4: 기본 AI

### 4-1. AIShipComponent
```
src/game/component/ai_ship.h
src/game/component/ai_ship.cpp
```

- 상태 머신: Idle → Pursue → Attack → Evade
- `on_fixed_update`에서:
  - 타겟 선택 (가장 가까운 Player 팩션 엔티티)
  - 추적: proportional navigation 또는 단순 pursuit
  - 사거리 진입 시 발사 (같은 `spawn_projectile()` 사용)
  - 회피: 피격 시 or 랜덤 기동

### 4-2. 궤도 역학과의 조합
- AI도 궤도 위에 있으므로, 단순 직선 추적은 안 됨
- 최소한의 접근: "물리 추력으로 타겟 방향 가속" (ShipController와 동일한 힘 적용)
- 고급: 호만 전이 or 램버트 솔버 기반 기동 계획 (Phase 4+)

---

## Phase 5: 이펙트 / 폴리시

### 5-1. 발사체 비주얼
- 트레이서 라인 (디버그 드로우 또는 전용 렌더)
- 포구 섬광 (파티클)
- 피탄 이펙트 (파티클 버스트)

### 5-2. 사운드
- 발사음 (miniaudio)
- 피격음
- 파괴음

### 5-3. 카메라 이펙트
- 피격 시 카메라 쉐이크
- 파괴 시 줌 아웃 or 슬로 모션

---

## 아키텍처 고려 사항

### 엔티티 파괴 타이밍
충돌 콜백 안에서 바로 `destroy_entity()`를 호출하면 재진입 문제가 생긴다.
→ 파괴 요청을 큐에 넣고, `on_fixed_update` 끝에 일괄 처리하는 패턴이 필요:

```cpp
// GameplayState or GameWorld
std::vector<EntityId> _pending_destroy;

void queue_destroy(EntityId id) { _pending_destroy.push_back(id); }

void flush_destroy() {
    for (auto id : _pending_destroy) {
        destroy_entity(id);
    }
    _pending_destroy.clear();
}
```

### 발사체와 부동 소수점 원점
발사체는 함선 근처에서 생성되므로 리베이싱 문제는 크지 않다.
다만 장거리 사격(수 km) 시 float 정밀도 문제 가능 → TTL로 자연 제한.
레일 워프 중에는 발사체 스폰을 막는 게 안전하다.

### 충돌 레이어 매트릭스
```
             Static  Dynamic  Kinematic  Player  Enemy  Projectile  Trigger  Debris
Static         -       O        -         O       O       O          -        O
Dynamic        O       O        O         O       O       O          O        O
Kinematic      -       O        -         O       O       O          O        -
Player         O       O        O         -       O       -          O        O
Enemy          O       O        O         O       -       O          O        O
Projectile     O       O        O         -       O       -          -        -
Trigger        -       O        O         O       O       -          -        -
Debris         O       O        -         O       O       -          -        -
```
핵심: Player↔Projectile X (자기 탄에 안 맞음), Projectile↔Projectile X (탄끼리 무시)

### 씬 리셋과 상태 정리
`setup_scene()`이 이미 전체 초기화를 하므로, 전투 리셋은 기존 경로를 그대로 쓸 수 있다.
다만 동적 스폰된 발사체/파편이 `_orbiters`에는 포함되지 않으므로, `_world.clear()`가 이들도 정리하는지 확인 필요.

---

## 우선순위

```
Phase 0 (씬 다듬기)  →  Phase 1 (발사체)  →  Phase 2 (데미지/HP)
         ↘                                       ↘
          Phase 3 (타겟팅/HUD)             →     Phase 4 (AI)
                                                    ↘
                                                  Phase 5 (이펙트)
```

Phase 0~2를 끝내면 "발사 → 피격 → 파괴"의 최소 전투 루프가 완성된다.
Phase 3~4를 더하면 플레이 가능한 전투 데모가 된다.
