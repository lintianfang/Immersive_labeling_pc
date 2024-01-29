#version 450

uniform vec4 selection_enclosing_sphere; //x,y,z,radius in view space

/*
//***** begin interface selection_xxxxxx.glsl
bool is_in_selection(in vec4 position);
//***** end interface selection_xxxxxx.glsl *********************************
*/

bool is_in_selection(in vec4 position){
	vec3 d = position.xyz-selection_enclosing_sphere.xyz;
	float radius = selection_enclosing_sphere.w;
	return (dot(d,d) < radius*radius);
}

bool raycast_selection(in vec3 origin,in vec3 direction, out float t_result){
	//TODO add implementation
	return false;
}

bool resolve_collision(in vec4 position, out vec4 new_position, out vec3 collision_normal){
	vec3 d = position.xyz-selection_enclosing_sphere.xyz;
	vec3 direction = normalize(d);
	// 0 = v+len(d)-r
	float v = selection_enclosing_sphere.w-length(d);
	new_position = vec4(v*direction + position.xyz,1.0);
	collision_normal = direction;
	return v > 0.0;
}