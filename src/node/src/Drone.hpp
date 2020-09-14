


class Drone{
    public:
        Drone();
        ~Drone();
        
        void move(long double x, long double y, long double z, long double t);
        
        bool is_armed();
        
        void arm();

        // Lands exactly to z=0 position
        void land();

        // Lands a different position
        void land(long double x, long double y, long double t);


    private:
        long double _posx, _posy, _posz; //These positions are likely to be changed by another data type
        

};