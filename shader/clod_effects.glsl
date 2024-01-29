#version 430

layout(std430, binding = 32) buffer ssLodColorMapBuffer{
	readonly restrict vec4 lod_color_map[];
};

layout(location = 2) uniform bool color_based_on_lods = false;

/*
//***** begin interface of clod_effects.glsl
void apply_vertex_color_effects(in vec4 color, inout vec4 color_out);
//***** end interface of clod_effects.glsl
*/

// color must contain lod information for this function to work
void apply_vertex_color_effects(in vec4 color, inout vec4 color_out) {
	if (color_based_on_lods) {
		uint level = uint(mod(color.a * 255, 128));
		color_out.xyz = lod_color_map[level].xyz;
	}
}