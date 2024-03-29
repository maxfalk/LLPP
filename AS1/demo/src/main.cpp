//#include "ped_agent.h"
#include "ped_model.h"
#include "MainWindow.h"
#include "ParseScenario.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QApplication>
#include <QTimer>
#include <thread>

#include <unistd.h>

#include "Timer.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstring>
#define SIMULATION_STEPS 1000000



int main(int argc, char*argv[]) { 
  Ped::Model model;
  Ped::IMPLEMENTATION mode = Ped::IMPLEMENTATION::SEQ;
  int threads = 4;
  bool timing_mode = 0;
  int i = 1;
  QString scenefile = "scenario.xml";

  // Argument handling
  while(i < argc) {
    if(argv[i][0] == '-' && argv[i][1] == '-') {
      if(strcmp(&argv[i][2],"timing-mode") == 0) {
        cout << "Timing mode on\n";
        timing_mode = true;
      } else {
        cerr << "Unrecognized command: \"" << argv[i] << "\". Ignoring ..." << endl;
      }
    } else if (argv[i][0] == '-') {
      if (strcmp(&argv[i][1], "mode") == 0) {
        char *input = argv[++i];
        if (strcmp(input, "PTHREAD") == 0) {
          mode = Ped::IMPLEMENTATION::PTHREAD;
	  cout << "PTHREAD mode selected." << endl;
        } else if (strcmp(input, "OMP") == 0) {
          mode = Ped::IMPLEMENTATION::OMP;
	  cout << "OMP mode selected." << endl;
        } else if (strcmp(input, "VECTOR") == 0) {
          mode = Ped::IMPLEMENTATION::VECTOR;
	  cout << "VECTOR mode selected." << endl;
	}
      } else if (strcmp(&argv[i][1], "threads") == 0) {
        threads = atoi(argv[++i]);
        if (threads < 1) {
          threads = 4;
	  cout << threads << " threads selected." << endl;
	}
      }
    } else { // Assume it is a path to scenefile
      scenefile = argv[i];
    }
    i+=1;
  }
 
  ParseScenario parser(scenefile);
  model.setup(parser.getAgents(), mode, threads);

  QApplication app(argc, argv);

  MainWindow mainwindow(model);




   const int delay_ms = 100;
   Timer *timer;
  #define TICK_LIMIT 10000
  #define AS_FAST_AS_POSSIBLE 0
   if(timing_mode)
   {
    timer = new Timer(model,mainwindow,AS_FAST_AS_POSSIBLE);
    timer->setTickLimit(TICK_LIMIT);
  }
  else
  {
    timer = new Timer(model,mainwindow,delay_ms);
    mainwindow.show();

  }
  cout << "Demo setup complete, running ..." << endl;
  int retval = 0;
  std::chrono::time_point<std::chrono::system_clock> start,stop;
  start = std::chrono::system_clock::now();

  // If timing mode, just go as fast as possible without delays or graphical updates
  if(timing_mode)
  {
    timer->busyTick();
  }
  else
  {
    timer->qtTimerTick();
    retval = app.exec();
  }

  stop = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = stop-start;
  cout << "Time: " << elapsed_seconds.count() << " seconds." << endl;

  cout << "Done" << endl;
  delete (timer);
  return retval;
}
