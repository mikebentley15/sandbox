#include "PlottableStream.h"


PlottableStream::PlottableStream(const std::string tabTitle)
	: mTabTitle(tabTitle) {}


std::string PlottableStream::getTitle() {

	return mTabTitle;

}
