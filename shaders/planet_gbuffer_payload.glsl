#ifndef PLANET_GBUFFER_PAYLOAD_GLSL
#define PLANET_GBUFFER_PAYLOAD_GLSL

// ---------------------------------------------------------------------------
// Planet G-Buffer pos.w encoding
// ---------------------------------------------------------------------------
// Packs planet terrain information into the G-Buffer position.w channel.
//
// pos.w layout:
//   0.0                  = background (no geometry)
//   1.0                  = regular object (valid pixel)
//   2.0 + frac           = planet surface (planet flag + payload)
//
// Payload (fractional part * 256 -> 8-bit unsigned integer):
//   [7:4] waterMask       — ocean mask       (4-bit, 0~15 -> 0.0~1.0)
//   [3:0] terrainSunVis   — terminator self-shadow visibility (4-bit, 0~15 -> 0.0~1.0)
// ---------------------------------------------------------------------------

const float PLANET_GBUFFER_VALID_BASE = 1.0;  // base value indicating a valid pixel
const float PLANET_GBUFFER_PLANET_FLAG = 1.0; // added to base for planet surfaces (1.0 + 1.0 = 2.0)

// Quantize a [0.0, 1.0] float to a 4-bit unsigned integer (0~15).
uint pack_planet_gbuffer_u4(float value)
{
    return uint(clamp(round(value * 15.0), 0.0, 15.0));
}

// Quantize waterMask and terrainSunVis into 4 bits each, then merge into an 8-bit payload.
uint pack_planet_gbuffer_payload(float waterMask, float terrainSunVis)
{
    const uint water_bits = pack_planet_gbuffer_u4(clamp(waterMask, 0.0, 1.0));
    const uint sun_bits = pack_planet_gbuffer_u4(clamp(terrainSunVis, 0.0, 1.0));
    return (water_bits << 4u) | sun_bits;
}

// Encode pos.w for the G-Buffer output.
// Planet surfaces (gbufferFlags > 0.5) store the payload in the fractional part.
// Non-planet geometry writes 1.0 with no payload.
float encode_planet_gbuffer_pos_w(float gbufferFlags, float waterMask, float terrainSunVis)
{
    float pos_w = PLANET_GBUFFER_VALID_BASE + clamp(gbufferFlags, 0.0, PLANET_GBUFFER_PLANET_FLAG);
    if (gbufferFlags > 0.5)
    {
        pos_w += float(pack_planet_gbuffer_payload(waterMask, terrainSunVis)) / 256.0;
    }
    return pos_w;
}

// Returns true if this pixel belongs to a planet surface (pos.w >= 2.0).
bool decode_planet_gbuffer_is_planet(float posW)
{
    return posW > (PLANET_GBUFFER_VALID_BASE + 0.5);
}

// Recover the 8-bit payload from the fractional part of pos.w.
// Returns 0 for non-planet pixels.
uint decode_planet_gbuffer_payload(float posW)
{
    if (!decode_planet_gbuffer_is_planet(posW))
    {
        return 0u;
    }
    return uint(clamp(round(fract(posW) * 256.0), 0.0, 255.0));
}

// Extract ocean mask [0.0, 1.0] from the upper 4 bits of the payload.
float decode_planet_gbuffer_water_mask(float posW)
{
    const uint payload = decode_planet_gbuffer_payload(posW);
    return float((payload >> 4u) & 0xFu) * (1.0 / 15.0);
}

// Extract terminator self-shadow visibility [0.0, 1.0] from the lower 4 bits.
// Returns 1.0 (fully lit) for non-planet pixels.
float decode_planet_gbuffer_terrain_sun_vis(float posW)
{
    if (!decode_planet_gbuffer_is_planet(posW))
    {
        return 1.0;
    }
    const uint payload = decode_planet_gbuffer_payload(posW);
    return float(payload & 0xFu) * (1.0 / 15.0);
}

#endif
