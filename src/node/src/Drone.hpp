


class Drone{
    public:
        Drone();
        ~Drone();
        
        void move(long double x, long double y, long double z, long double t);

        //Here x,y,z gives us the center of the circle, r is radius, t is the same as the t in move function.
        void circular_move(long double x, long double y, long double z, long double t, long double r);
        
        bool is_armed();
        
        void arm();

        // Lands exactly to z=0 position
        void land();

        // Lands a different position
        void land(long double x, long double y, long double t);


    private:
        long double _posx, _posy, _posz; //These positions are likely to be changed by another data type
        

};