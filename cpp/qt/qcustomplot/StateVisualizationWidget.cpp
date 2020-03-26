#include "StateVisualizationWidget.h"
#include <QWidget>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include "qcustomplot.h"
#include <vector>
#include <iostream>
#include "Timer.h"


StateVisualizationWidget::StateVisualizationWidget(const std::vector<PlottableStream*>& plottableStreams, int timestep_ms, int windowSize, QWidget* parent)
	: QTabWidget(parent),
	  mPlottableStreams(plottableStreams),
	  windowSize(windowSize) {

	updateTimer = new QTimer(this);

	// Create the default pen list
	defaultPenList.push_back(QPen(Qt::blue));
	defaultPenList.push_back(QPen(Qt::green));
	defaultPenList.push_back(QPen(Qt::red));
	defaultPenList.push_back(QPen(Qt::darkBlue));
	defaultPenList.push_back(QPen(Qt::darkGreen));
	defaultPenList.push_back(QPen(Qt::darkRed));
	defaultPenList.push_back(QPen(Qt::magenta));
	defaultPenList.push_back(QPen(Qt::cyan));
	defaultPenList.push_back(QPen(Qt::yellow));
	defaultPenList.push_back(QPen(Qt::black));

	unsigned long defaultPenIndex = 0;

	// Create a tab for each mPlottableStreams
	for (unsigned long streamIndex = 0; streamIndex < mPlottableStreams.size(); streamIndex++) {

		// Get the plotting attributes
		std::vector<PlotAttributes> attributes;
		mPlottableStreams[streamIndex]->getPlottingAttributes(attributes);

		// Create plots, add the graphs to the plots and put them in a layout
		QVBoxLayout* layout = new QVBoxLayout();
		for (unsigned long streamPlotIndex = 0; streamPlotIndex < attributes.size(); streamPlotIndex++) {

			// Add a title to the plot
			QString title = "<b>";
			title.append(attributes[streamPlotIndex].name.c_str());
			title.append("</b>");
			QLabel* plotTitle = new QLabel(title);
			plotTitle->setTextFormat(Qt::RichText);
			plotTitle->setAlignment(Qt::AlignCenter);
			plotTitle->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed));
			layout->addWidget(plotTitle);

			QCustomPlot* plot = new QCustomPlot();

			// Configure the graphs for this plot
			for (int graphIndex = 0; graphIndex < static_cast<int>(attributes[streamPlotIndex].graphAttributes.size()); graphIndex++) {

				// Add the graph
				plot->addGraph();

				GraphAttributes graphAttributes = attributes[streamPlotIndex].graphAttributes[static_cast<unsigned long>(graphIndex)];

				// Set the name of the graph for the legend
				plot->graph(graphIndex)->setName(graphAttributes.name.c_str());

				// Set the pen color for the graph
				plot->graph(graphIndex)->setPen(defaultPenList[defaultPenIndex++]);

				if (defaultPenIndex >= defaultPenList.size())
					defaultPenIndex = 0;

				// Set the x-axis and y-axis ranges of the graph?


			}

			// reset default plot colors
			defaultPenIndex = 0;

			// Make the legend visible and place it in the top right of the plot if there is more than one graph for this plot.
			if (attributes[streamPlotIndex].graphAttributes.size() > 1) {
				QCPLayoutGrid* subLayout1 = new QCPLayoutGrid;
				plot->plotLayout()->addElement(0, 1, subLayout1);
				subLayout1->addElement(0, 0, new QCPLayoutElement);
				subLayout1->addElement(1, 0, plot->legend);
				subLayout1->addElement(2, 0, new QCPLayoutElement);
				plot->plotLayout()->setColumnStretchFactor(1, 0.001);
				plot->legend->setBorderPen(QPen(Qt::transparent));
				plot->legend->setVisible(true);
			}

			// Set the x-axis of the plot to use a time format (time data must be in seconds)
			QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
			timeTicker->setTimeFormat("%m:%s");
			plot->xAxis->setTicker(timeTicker);
			plot->xAxis->setLabel("Time");

			plot->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));
			plot->setMinimumSize(QSize(800, plot->legend->minimumOuterSizeHint().height() + 100));

			plots.push_back(plot); // Store the plot for later access
			layout->addWidget(plot); // Add the plot to the layout

		}

		// Add the layout to a QWidget
		QWidget* tab = new QWidget();
		tab->setLayout(layout);

		QScrollArea* scrollArea = new QScrollArea();
		scrollArea->setWidget(tab);
		scrollArea->setWidgetResizable(true);
		scrollArea->setBackgroundRole(QPalette::Light);

		// Add the QWidget as a tab
		addTab(scrollArea, mPlottableStreams[streamIndex]->getTitle().c_str());

	}

	connect(updateTimer, SIGNAL(timeout()), this, SLOT(updatePlots()));

	startTime_ms = Timer::getSysTime();

	updateTimer->start(timestep_ms);

}


void StateVisualizationWidget::updatePlots() {

	// Used to keep track of which plot (overall) we are currently updating
	unsigned long totalPlotIndex = 0;

	double time = (Timer::getSysTime() - startTime_ms) / 1000.0;

	// Iterate through all the mPlottableStreams
	for (unsigned long streamIndex = 0; streamIndex < mPlottableStreams.size(); streamIndex++) {

		// Get the data in plotting format
		PlottingFormat plottingData;
		mPlottableStreams[streamIndex]->getDataInPlottingFormat(plottingData);

		// Iterate over each plot for this stream
		for (unsigned long streamPlotIndex = 0; streamPlotIndex < plottingData.stateValues.size(); streamPlotIndex++) {

			// Add the data points to each graph for this plot
			for (unsigned long graphIndex = 0; graphIndex < plottingData.stateValues[streamPlotIndex].size(); graphIndex++)
				plots[totalPlotIndex]->graph(static_cast<int>(graphIndex))->addData(time, plottingData.stateValues[streamPlotIndex][graphIndex]);

			// Rescale the y-axis
			plots[totalPlotIndex]->yAxis->rescale();

			// Update the range of the time axis to give window following behavior
			plots[totalPlotIndex]->xAxis->setRange(time, windowSize, Qt::AlignRight);

			// Update the plot on the screen
			plots[totalPlotIndex]->replot();

			totalPlotIndex++;

		}

	}

}
