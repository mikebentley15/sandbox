#ifndef STATEVISUALIZATIONWIDGET_H
#define STATEVISUALIZATIONWIDGET_H

#include <QWidget>
#include <QTabWidget>
#include <QTimer>
#include "qcustomplot.h"
#include "PlottableStream.h"
#include <vector>


class StateVisualizationWidget : public QTabWidget {

	Q_OBJECT

	private:
		std::vector<PlottableStream*> mPlottableStreams;
		std::vector<QCustomPlot*> plots;

		std::vector<QPen> defaultPenList; // Default order of graph colors to cycle through when the user doesn't specify a QPen

		int windowSize;
		long startTime_ms;

		QTimer* updateTimer;

	public slots:
		void updatePlots();

	signals:
		void closing(QWidget*);

	public:
		StateVisualizationWidget(const std::vector<PlottableStream*>& plottableStreams, int timestep_ms = 50, int windowSize = 40, QWidget* parent = nullptr);

};

#endif // STATEVISUALIZATIONWIDGET_H
