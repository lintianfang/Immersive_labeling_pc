#pragma once
#include <cgv/utils/tokenizer.h>
#include <iostream>
#include <unordered_map>
#include <string>
#include <regex>

#include <cgv/media/color.h>

enum class point_label_group {
	DELETED = 0x0000,	//deleted points must not be in any group
	VISIBLE = 0x0001,	//meber of first group with normal clod rendering (the default group) and label 0
	SELECTED_BIT = 0x0002,  //points with a special highlighting
	PROTECTED_BIT = 0x0004, //points that are excluded from regular labeling
	GROUP_MASK = 0xFFFF,// there are 16 point groups. A point can be in all of them simultaneously, groups can be used to better implement selection mechanisms
	UNDEFINED = -1
};

enum class point_label_operation {
	REPLACE = 0,
	OR = 1,
	AND = 2,
	NONE = -1
};

/// encodes a number as label
inline constexpr uint32_t make_label(const uint32_t label_id, const point_label_group& group) {
	return (label_id << 16) | ((uint32_t)group);
}

inline constexpr uint32_t make_label(const uint32_t label_id, const uint32_t group) {
	return (label_id << 16) | (group & (uint32_t)point_label_group::GROUP_MASK);
}

/// decodes a number from a label by removing the group information
inline constexpr uint32_t remove_label_group(uint32_t label) {
	return label >> 16;
}

inline uint32_t get_label_group(uint32_t label) {
	return label & (uint32_t)point_label_group::GROUP_MASK;
}

std::unordered_map<unsigned int, std::string> parse_label_mappings(std::istream& is);


void parse_label_mappings(std::istream& is, std::unordered_map<unsigned int, std::string>* name_mappings, std::unordered_map<unsigned int, cgv::media::color<float>>* color_mappings);