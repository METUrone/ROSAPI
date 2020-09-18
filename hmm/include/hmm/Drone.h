

class Drone{
    public:
        Drone(){};
        ~Drone(){};
	int i =2;

    private:
        long double _posx, _posy, _posz; //These positions are likely to be changed by another data type
        void move(long double x, long double y, long double z, long double t);

};
