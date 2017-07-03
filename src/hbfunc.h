void Nutrients_add(int amount);
typedef struct {
    int delay;
    int target_ph;
} vPH_parameters;

typedef struct {
    int status;
    int delay;
} vLight_parameters;


int Plant_get_height();

void PH_up_add(int difference) {
    int water_level;
    //find volume and find how much ph add we need
}

void PH_down_add(int difference) {

    //adds 

}

int PH_get_value() {
    int ph_value;
    return ph_value;
}



void Light_set(int status) {
    /*
     * Params:
     *  -> status (bool)
    */
}

void Light_distance_set(int distance) {
    //set light distance from plant 
}

int Light_get_status() {
    int status=0; //on or off
    return status;
}

