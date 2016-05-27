#define NANO 1000000000L


class Temps{
  
  private:
  
  struct timespec t_start, t_stop;
  int clk_id;
  
  
  public:
    
  Temps(); //constructor

  void tic();
  float tac();
  
  ~Temps(); //destructor
};


Temps::Temps()
{
clk_id = 1;
}


    
void Temps::tic()
{
  clock_gettime(clk_id, &t_start);

}


float Temps::tac()
{
  clock_gettime(clk_id, &t_stop);
  
  unsigned long time = (t_stop.tv_sec * NANO + t_stop.tv_nsec) - (t_start.tv_sec * NANO + t_start.tv_nsec);  

  //printf("\nTemps écoulé entre tic et tac: %ld µs\n", time/1000);

return time/1000;

}

Temps::~Temps()
{
}
