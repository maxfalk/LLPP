#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>
#include "ped_crowd.h"
#include "ped_net.h"

#define COL_THREADS 4
#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE*CELLSIZE

namespace Ped{

  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Crowd*> crowdsInScenario, IMPLEMENTATION mode, 
	       int nrOfThreads, bool parallelCollision, bool heatMap);
    void tick();
    const std::vector<Crowd*> getCrowds() const;

    //~Model();
    int getHeatmapSize() const;
    int const * const * getHeatmap() const;

  private:
    int nrOfThreads;
    IMPLEMENTATION implementation;
    std::vector<Crowd*> crowds;
    bool parallelCollision;
    bool heatmapPar;
    void omp();
    void seq();
    void pThreads();
    void vector();
    void cuda();

    static void doSafeMovement(Net::Npair Agent);
    static void doSafeMovementParallel(Net::Npair Agent);    
    static void *checkCollisions(void *data);
    void create_threads(int nrOfThreads);
    static void *threaded_tick(void *inds);


    int cudaThreads;
    int **heatmap;
    int **scaled_heatmap;
    static int **blurred_heatmap;

    void start_setup_heat_map();
    static void *updateHeatmapPar(void *_notused);
    void calc_max_cuda_threads(int);

    void setupHeatmapSeq();
    void updateHeatmapSeq();


    //test
    void assertAgents();

  };
  
}

#endif
