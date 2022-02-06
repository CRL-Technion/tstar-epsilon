//
// Created by doron.pinsky on 29/01/2020.
//
#ifndef ROBOT_H
#define ROBOT_H

class Robot {
    public:
        Robot();
        void setuMax(double uMaxIn);
        void setvMax(double vMaxIn);
        void setvMin(double vMinIn);
        void setEta(double etaIn);
        void setNumOfOreintations(int orientationIn);
        void setSpeed(int speed);
        int getSpeed();
        double getuMax();
        double getvMax();
        double getvMin();
        double getEta();
        double getStepSize();
        double get_r();
        double get_R();
        int getNumOfOrientation();
        void printInfo();
		
		void setOcpsStatus(bool ocps);
		bool getOcpsStatus(); 
    
    private:
        double uMax;
        double vMax;
        double vMin;
        double m_r;
        double m_R;
        double eta;
        int numOfOreintations;
        double m_stepsize;
        int m_speed; //0-low, 1-high
		
		bool _ocpsStatus = false;
};

#endif //ROBOT_H
