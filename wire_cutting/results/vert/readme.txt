Hver mappe indeholder forskellige indstilling til samme bane.

Filen: 
log/wire_cutting_wire_cutting_node-3-stdout 
Der ligger i hver mappe er den mest interresante. Den indeholder costs og constaints for hvert tidsskridt.

Ingen af eksemplerne udnytte translation langs wiren

De constraints der hedder "user_defined_INEQ" er akse grænser fra det parallelle link mellem akse 2 og 3

To forskellige initialiseringer bliver brugt:
Kompleks:
   * @brief CartesianWaypoint to CartesianWaypoint
   *
   * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
   *
   * - the number of steps for the plan will be calculated such that:
   *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
   *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
   *   - the number of steps for the plan will be calculated such that the norm of all joint distances between
   *successive steps is no longer than state_longest_valid_segment_length
   *   - the max steps from the above calculations will be be compared to the min_steps and the largest will be chosen
   * - the interpolation will be done based on the condition below
   *   - Case 1: Joint solution found for start and end cartesian waypoint
   *     - It interpolates the joint position from the start to the end state
   *   - Case 2: Joint solution only found for start cartesian waypoint
   *     - It creates number states based on the steps and sets the value to found start solution
   *   - Case 3: Joint solution only found for end cartesian waypoint
   *     - It creates number states based on the steps and sets the value to found end solution
   *   - Case 4: No joint solution found for end and start cartesian waypoint
   *     - It creates number states based on the steps and sets the value to the current state of the environment
   *
   
Naiv:
Initialiserer alle tidsskridt i den state robotten står i.


def_seed_abs_rot_cost:
Bruger kompleks initialisering.
Absolut rotations cost for rotation omkring wiren
Fejler

def_seed_no_rot_cost:
Bruger kompleks initialisering
Ingen rotations cost for rotation omkring wiren
Når løsning

def_seed_sq_rot_cost:
Bruger kompleks initialisering.
Squared rotations cost for rotation omkring wiren
Når løsning

naive_seed_abs_rot_cost:
Bruger naiv initialisering
Absolut rotations cost for rotation omkring wiren
Når løsning



