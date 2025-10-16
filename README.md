# New-Live-Loadcase-Solver
In this folder is the script ‘New_Live_Loadcase_Solver’, as well as several functions that take in the suspension bearing locations (hardpoints, or hPoints) from hardpoints.csv, car parameters such as weight, CoG height etc’ and the velocity in x and acceleration in x, y, and z for different loading conditions, such as accelerating, braking etc. For each loading condition, it returns the forces in each suspension member, and the forces acting on the upright at the track rod, and upper and lower wishbone mounting points, as well as the max compressive and tensile force each suspension member will see at any time.

The suspension bearing locations in hardpoints.csv are arranged:

|                        | x | y | z |
|------------------------|---|---|---|
| Front RHS LWR OB       |   |   |   |
| Front RHS LWR FWD IB   |   |   |   |
| Front RHS LWR RWD IB   |   |   |   |
| Front RHS UPR OB       |   |   |   |
| Front RHS UPR FWD IB   |   |   |   |
| Front RHS UPR RWD IB   |   |   |   |
| Front RHS PR OB        |   |   |   |
| Front RHS PR IB        |   |   |   |
| Front RHS TR OB        |   |   |   |
| Front RHS TR IB        |   |   |   |
| Front RHS Wheel Centre |   |   |   |
| Rear RHS LWR OB        |   |   |   |
| Rear RHS UPR FWD IB    |   |   |   |
| Rear RHS LWR RWD IB    |   |   |   |
| Rear RHS UPR OB        |   |   |   |
| Rear RHS UPR FWD IB    |   |   |   |
| Rear RHS UPR RWD IB    |   |   |   |
| Rear RHS PR IB         |   |   |   |
| Rear RHS TR OB         |   |   |   |
| Rear RHS TR IB         |   |   |   |
| Rear RHS Wheel Centre  |   |   |   |

Where x extends rearwards from a vertical plane at the front axle centre line, y extends to the RHS of the car from a vertical plane at the longitudinal centreline of the car, and z extends upwards from a horizontal plane at the tyre contact patches. If taking coordinates from NX, z is longitudinal, x is horizontal and y is vertical. hardpointsParameters.m can convert the NX coordinate system to the coordinate system used by the rest of the code. 
