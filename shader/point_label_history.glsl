#version 450

struct label_operation {
	int label;
	uint index;
};

layout(std430, binding = 7) buffer ssLabelHistory
{
    label_operation history[];
};

layout(std430, binding = 8) buffer ssLabelHistoryMetaData
{
    uint history_start;
	uint history_capacity; // allways (power of 2) -1, allows to use & as replacement for mod
	uint points_written; // points_written > capacity means the ringbuffer overflowed and the history got corrupted
};


/*
//***** begin interface of point_label_history.glsl **********************************
struct label_operation {
	int label;
	uint index;
};
void store_point_label(int label, uint point_id);
label_operation get_operation(uint local_ix);
uint get_operation_size();
//***** end interface of point_label_history.glsl **********************************
*/


uint global_index_of(uint local_ix) {
	return (local_ix + history_start) & history_capacity;
}

uint get_operation_size(){
	return points_written;
}

void store_point_label(int label, uint point_id){
	uint ix = global_index_of(atomicAdd(points_written, 1));
	history[ix].label = label;
	history[ix].index = point_id;
}

// local_ix referes to the index inside the operation
// e.g local_ix=0 referes to the first point label stored at history_start
// add_rollback_operation() on host side will modify history_start and the written points count
label_operation get_operation(uint local_ix){
	return history[global_index_of(local_ix)]; 
}