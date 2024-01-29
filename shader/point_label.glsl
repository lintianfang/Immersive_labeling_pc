#version 450

//this shader contains helper functions for assigning labels to points

// uses the same binding point as the custom label shader for point labels
layout(std430, binding = 6) buffer ssLabels
{
    restrict int labels[];
};

#define POINT_VISIBLE 1
#define POINT_DELETED 0
#define POINT_GROUP_MASK 0x0000FFFF
#define POINT_GROUP_BITS 16
#define POINT_LABEL_MASK 0xFFFF0000

/*
//***** begin interface of point_label.glsl **********************************
void label_point(in uint index, in int point_label, in int point_group_mask);
void label_point(in uint index, in int point_label);
int get_label(in uint index);
bool label_is_in_group_selection(in int label, in int point_group_mask, in int exclude_point_group_mask);
//***** end interface of point_label.glsl **********************************
*/

#define UNDEFINED_INDEX -1

//sets a point label
void label_point(in uint index, in int point_label) {
	labels[index] = point_label;
}

//reads a point label and checks the index
int get_label(in uint index){
	return labels[index];
}

//sets the label of a point if the point belongs to one of the groups covered by the point_group_mask
void label_point(in uint index, in int point_label, in int point_group_mask){
	if ( (get_label(index) & point_group_mask) != 0){
		label_point(index,point_label);
	}
}

bool label_is_in_group_selection(in int label, in int point_group_mask, in int exclude_point_group_mask){
	return ((label & point_group_mask) != 0) && ((label & exclude_point_group_mask) == 0);
}

bool point_is_in_group_selection(in uint index, in int point_group_mask, in int exclude_point_group_mask){
	int label = get_label(index); 
	return ((label & point_group_mask) != 0) && ((label & exclude_point_group_mask) == 0);
}