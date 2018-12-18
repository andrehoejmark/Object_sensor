// stepper.h
static unsigned char fullStep[] = {4, 0b1000, 0b0010, 0b0100, 0b0001};
static unsigned char halfStep[] = {8, 0b1000, 0b1010, 0b0010, 0b0110, 0b0100, 0b0101, 0b0001, 0b1001};

class StepperMotor{
  private:
    unsigned char coils[4];

    unsigned char *stateMachine;
    unsigned char state;
    unsigned int stepDelay;
    unsigned int rotationDelay;
    
    void outputState();
    void outputPin(unsigned char pin, unsigned char mode);

  public:
    enum LR {left, right};
    StepperMotor(unsigned char *stateMachine, unsigned char *coils, unsigned int stepDelay, unsigned int rotationDelay);
    ~StepperMotor();
    void Stepp(unsigned int stepps, LR dir);
    void SteppSingle(LR dir);
    void activate(unsigned char state);
    void deacivate();
    
    
    void steppRight(unsigned int stepps){
      this->Stepp(stepps, right);
    }
    void steppLeft(unsigned int stepps){
      this->Stepp(stepps, left);
    }
    void steppRight(){
      this->SteppSingle(right);
    }
    void steppLeft(){
      this->SteppSingle(left);
    }
};
