#ifndef PLOTTABLESTREAM_H
#define PLOTTABLESTREAM_H

#include <string>
#include <vector>

struct GraphAttributes {

	std::string name;

};


struct PlotAttributes {

	std::string name;
	std::vector<GraphAttributes> graphAttributes;

};


struct PlottingFormat {

	std::vector<std::vector<double>> stateValues;

};


class PlottableStream {

	private:
		std::string mTabTitle;

	public:
		PlottableStream(const std::string tabTitle);
		virtual ~PlottableStream() = default;

		virtual void getPlottingAttributes(std::vector<PlotAttributes>& attributes) = 0;
		virtual void getDataInPlottingFormat(PlottingFormat& data) = 0;
		std::string getTitle();

};

#endif // PLOTTABLESTREAM_H
