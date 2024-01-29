#include "point_labels.h"

#include <cgv/render/render_types.h>
#include <regex>


void parse_label_mappings(std::istream& is, std::unordered_map<unsigned int, std::string>* name_mappings, std::unordered_map<unsigned int, cgv::media::color<float>>* color_mappings) {
	std::string line;
	std::vector<cgv::utils::token> results;
	std::regex label_value_pair = std::regex("[0-9]*:[a-zA-Z]*");
	std::regex assignment_op = std::regex("=");
	
	while (is.good()) {
		getline(is, line);
		std::smatch m;
		//find "=" and split string
		auto assignment_pos = line.find('=');
		if (assignment_pos >= line.size()) {
			continue;
		}
		//line = line.substr(0,line.find_first_of("\n"));
		auto part_a = line.substr(0, assignment_pos);
		auto part_b = line.substr(assignment_pos+1, line.size()- (assignment_pos + 1));
		if (std::regex_search(part_a, m, label_value_pair)) {
			auto kvp_token = m[0].str();

			auto col_pos = kvp_token.find(":");
			unsigned int label = std::stoi(kvp_token.substr(0, col_pos));
			std::string value = kvp_token.substr(col_pos + 1, kvp_token.length());
			if (value == "name") {
				(*name_mappings)[label] = part_b;
			}
			else if (value == "color") {
				int32_t color = stol(part_b, 0, 16);
				(*color_mappings)[label] = cgv::render::render_types::rgb8((color>>16)&0xFF,(color>>8)&0xFF,color&0xFF);
			}
			
		}
		
	}
}

std::unordered_map<unsigned int, std::string> parse_label_mappings(std::istream& is) {
	std::string line;
	std::vector<cgv::utils::token> results;
	std::unordered_map<unsigned int, std::string> ret;
	while (is.good()) {
		getline(is, line);
		results.clear();
		bite_all(cgv::utils::tokenizer(line).set_sep(":,").set_skip("\"", "\"", "\\"), results);
		try {
			if (results.size() < 3) {
				//to small
				break;
			}
			auto label = std::stol(to_string(results[0]));
			if (results[1] == ":") {
				std::string label_name = to_string(results[2]);
				unsigned label = std::stoi(to_string(results[0]));
				if (label_name.front() == '\"' && label_name.back() == '\"') {
					ret[label] = label_name.substr(1, label_name.size() - 2);
				}
				else {
					ret[label] = label_name;
				}
			}
			else {
				//parse error
				break;
			}
		}
		catch (std::invalid_argument except) {

		}
	}
	return ret;
}