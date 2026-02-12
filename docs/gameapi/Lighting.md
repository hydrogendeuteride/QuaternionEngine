# Lighting System

Manage directional (sun), point, and spot lights with both single and double-precision support.

## Directional Light (Sunlight)

### Direction

```cpp
// Set sun direction (normalized automatically)
api.set_sunlight_direction(glm::vec3(0.3f, -0.8f, 0.5f));

// Query current direction
glm::vec3 dir = api.get_sunlight_direction();
```

The direction vector points **toward** the sun (opposite of light direction).

### Color and Intensity

```cpp
// Warm daylight
api.set_sunlight_color(glm::vec3(1.0f, 0.95f, 0.8f), 1.0f);

// Bright noon
api.set_sunlight_color(glm::vec3(1.0f, 1.0f, 1.0f), 1.5f);

// Sunset
api.set_sunlight_color(glm::vec3(1.0f, 0.6f, 0.3f), 0.8f);

// Query current settings
glm::vec3 color = api.get_sunlight_color();
float intensity = api.get_sunlight_intensity();
```

## Point Lights

### Data Structures

```cpp
// Single precision
struct PointLight
{
    glm::vec3 position{0.0f};
    float radius{10.0f};         // Attenuation radius
    glm::vec3 color{1.0f};       // RGB color
    float intensity{1.0f};       // Brightness multiplier
};

// Double precision (for large worlds)
struct PointLightD
{
    glm::dvec3 position{0.0};    // Double-precision world position
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
};
```

### Adding Point Lights

```cpp
// Single precision
GameAPI::PointLight light;
light.position = glm::vec3(0.0f, 3.0f, -5.0f);
light.radius = 15.0f;
light.color = glm::vec3(1.0f, 0.9f, 0.7f);  // Warm white
light.intensity = 100.0f;

size_t idx = api.add_point_light(light);

// Double precision
GameAPI::PointLightD lightD;
lightD.position = glm::dvec3(1000000.0, 10.0, 500000.0);
lightD.radius = 50.0f;
lightD.color = glm::vec3(1.0f, 0.5f, 0.2f);  // Orange
lightD.intensity = 200.0f;

size_t idxD = api.add_point_light(lightD);
```

Returns the light's index in the light array.

### Querying Point Lights

```cpp
// Single precision
GameAPI::PointLight light;
if (api.get_point_light(idx, light)) {
    fmt::println("Position: {}, {}, {}", light.position.x, light.position.y, light.position.z);
}

// Double precision
GameAPI::PointLightD lightD;
if (api.get_point_light(idx, lightD)) {
    // ...
}
```

### Updating Point Lights

```cpp
// Modify and update
GameAPI::PointLight light;
api.get_point_light(idx, light);
light.intensity = 50.0f;  // Dim the light
api.set_point_light(idx, light);
```

### Removing Point Lights

```cpp
bool removed = api.remove_point_light(idx);
```

### Clearing All Point Lights

```cpp
api.clear_point_lights();

// Query count
size_t count = api.get_point_light_count();
```

## Spot Lights

### Data Structures

```cpp
// Single precision
struct SpotLight
{
    glm::vec3 position{0.0f};
    glm::vec3 direction{0.0f, -1.0f, 0.0f};  // Normalized direction
    float radius{10.0f};                      // Attenuation radius
    glm::vec3 color{1.0f};
    float intensity{1.0f};
    float inner_angle_deg{15.0f};             // Inner cone (full brightness)
    float outer_angle_deg{25.0f};             // Outer cone (falloff to 0)
};

// Double precision
struct SpotLightD
{
    glm::dvec3 position{0.0};
    glm::vec3 direction{0.0f, -1.0f, 0.0f};
    float radius{10.0f};
    glm::vec3 color{1.0f};
    float intensity{1.0f};
    float inner_angle_deg{15.0f};
    float outer_angle_deg{25.0f};
};
```

**Note:** `inner_angle_deg` must be ≤ `outer_angle_deg`.

### Adding Spot Lights

```cpp
// Single precision
GameAPI::SpotLight spot;
spot.position = glm::vec3(0.0f, 5.0f, 0.0f);
spot.direction = glm::vec3(0.0f, -1.0f, 0.0f);  // Down
spot.radius = 20.0f;
spot.color = glm::vec3(1.0f, 1.0f, 0.9f);
spot.intensity = 150.0f;
spot.inner_angle_deg = 10.0f;
spot.outer_angle_deg = 25.0f;

size_t idx = api.add_spot_light(spot);

// Double precision
GameAPI::SpotLightD spotD;
spotD.position = glm::dvec3(2000000.0, 100.0, 1000000.0);
spotD.direction = glm::vec3(0.0f, -1.0f, 0.0f);
spotD.radius = 500.0f;
spotD.color = glm::vec3(1.0f, 0.95f, 0.8f);
spotD.intensity = 5000.0f;
spotD.inner_angle_deg = 5.0f;
spotD.outer_angle_deg = 15.0f;

size_t idxD = api.add_spot_light(spotD);
```

### Querying Spot Lights

```cpp
// Single precision
GameAPI::SpotLight spot;
if (api.get_spot_light(idx, spot)) {
    // ...
}

// Double precision
GameAPI::SpotLightD spotD;
if (api.get_spot_light(idx, spotD)) {
    // ...
}
```

### Updating Spot Lights

```cpp
GameAPI::SpotLight spot;
api.get_spot_light(idx, spot);
spot.direction = camera_forward;  // Follow camera
api.set_spot_light(idx, spot);
```

### Removing Spot Lights

```cpp
bool removed = api.remove_spot_light(idx);
```

### Clearing All Spot Lights

```cpp
api.clear_spot_lights();

// Query count
size_t count = api.get_spot_light_count();
```

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Setup sun
api.set_sunlight_direction(glm::vec3(0.3f, -0.8f, 0.5f));
api.set_sunlight_color(glm::vec3(1.0f, 0.95f, 0.85f), 1.0f);

// Add torch lights around level
for (const glm::vec3& pos : torchPositions) {
    GameAPI::PointLight torch;
    torch.position = pos;
    torch.radius = 8.0f;
    torch.color = glm::vec3(1.0f, 0.7f, 0.4f);  // Warm fire
    torch.intensity = 50.0f;
    api.add_point_light(torch);
}

// Add flashlight spot light
GameAPI::SpotLight flashlight;
flashlight.position = camera_position;
flashlight.direction = camera_forward;
flashlight.radius = 30.0f;
flashlight.color = glm::vec3(1.0f, 1.0f, 0.95f);
flashlight.intensity = 100.0f;
flashlight.inner_angle_deg = 8.0f;
flashlight.outer_angle_deg = 20.0f;

size_t flashlightIdx = api.add_spot_light(flashlight);

// Game loop: update flashlight to follow camera
while (running) {
    GameAPI::SpotLight flashlight;
    if (api.get_spot_light(flashlightIdx, flashlight)) {
        flashlight.position = camera_position;
        flashlight.direction = camera_forward;
        api.set_spot_light(flashlightIdx, flashlight);
    }
}
```

## Use Cases

### Flickering Lights

```cpp
// Update light intensity with noise/sine wave
GameAPI::PointLight light;
api.get_point_light(torchIdx, light);

float flicker = 1.0f + 0.15f * std::sin(time * 12.0f + noise);
light.intensity = baseIntensity * flicker;

api.set_point_light(torchIdx, light);
```

### Pulsing Warning Lights

```cpp
GameAPI::PointLight alarm;
api.get_point_light(alarmIdx, alarm);

// Pulse red every 0.5 seconds
float pulse = (std::sin(time * 4.0f * 3.14159f) + 1.0f) * 0.5f;
alarm.color = glm::vec3(1.0f, 0.0f, 0.0f);
alarm.intensity = 100.0f * pulse;

api.set_point_light(alarmIdx, alarm);
```

### Vehicle Headlights

```cpp
// Left and right headlight spots
GameAPI::SpotLight leftHeadlight;
leftHeadlight.position = vehicle_position + vehicle_left * 0.8f + vehicle_forward * 2.0f;
leftHeadlight.direction = vehicle_forward;
leftHeadlight.radius = 50.0f;
leftHeadlight.intensity = 200.0f;
leftHeadlight.inner_angle_deg = 10.0f;
leftHeadlight.outer_angle_deg = 30.0f;

api.set_spot_light(leftIdx, leftHeadlight);
// ... similar for right headlight
```

## Performance Considerations

- The engine supports many point and spot lights via clustered/tiled lighting
- Lights are culled per-tile during the lighting pass
- There's no hard limit, but expect performance to degrade with thousands of lights
- For best performance, prefer fewer, larger lights over many small lights

## See Also

- [Shadows](Shadows.md) — Shadow casting from directional light
- [IBL](IBL.md) — Image-based ambient lighting
- [Multi-Light System](../MultiLighting.md) — Low-level implementation details
