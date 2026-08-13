## Subject Creation Fix for Vicon Orientation Issue

**Issue:** 
1. The subjects tracked in by vicon had accurate translation, but totally inaccurate orienation

2. Orienation readings recorded when the subject was removed and put back to the same pose were inconsistent

3. Multiple subjects created while facing the same direction showed different orientation values

**Possible Reasoning:**
- Subjects' axes were not aligned with vicon origin axis. The vicon origin by default has the following axes convention:
    - +x: right
    - +y: forward
    - +z: up
- However, when creating a subject in vicon using markers laid out on a 2D plane, the primary and secondary axes are the z and y axes respectively thus creating the zy plane. Thus, the x axis is orthogonal to this plane and therefore up or down instead of right or left leading to all axes out of alignment wrt the world. You should automatically go offine now.
- This causes the difference in the axes orienation between vicon and the subject. Although it is plausible to think that the system should be able to calculate the transform between the subject frame and the world frame to result in the correct orientation values, this however did not happen. 

**Fix:**
- Using 2 reference "dummy" markers to allow us to firstly create a "dummy" subject that has markers in a 3D plane. 
- Then using this "dummy" subject to create our required subject with axes properly aligned with the world frame. 

This guide explains how a vicon subject of the Turtlebot4 robot has been created to correct the issue faced with orientation along the axes.
*Disclaimer*: The purpose of all steps is not fully understood, but following them leads to accurate creation of the subject to achieve correct orientation.

1. The reference markers have been placed around the subject. In this case the reference markers have been placed further along +y and +x than the subject in the world frame. Calibration may be slightly if done differently. 

2. Start the vicon system and go live

3. Place the subject in the field of view ideally along the arena axis (not rotated at an angle) as this will have an influence on your axes orienation. Make sure your subject markers and the reference markers are visible in the viewer. They should be white.

4. Under the Subjects tab on the left side bar, click on "Create a blank subject" and name it "TS".

5. Under the Subject Preparation tool on the right side bar, select "TS" from the Subject dropdown.

6. Save using Ctrl + S

7. Under Subject Capture, click Start and capture about 200 frames and then Stop. 

8. Under Subject Calibration, select the Pipeline as Reconstruct and click Start. You should automatically go offine now.

9. Under Labelling Template Builder, name the segement "TS" under Create Segments and click Create.

10. Now follow the marker selection order **carefully**:
    - The first/primary marker should be the marker in the centre of the subject
    - The primary axis marker (second selection) should be one of the markers placed for axis reference
    - The third selection (secondary axis marker) should be the other marker placed for axis reference
    - Select the remaining markers in any order and click Create

11. Save the subject and while your still offline navigate to Pipeline tab on the left side bar and locate:
    - Run Dynamic Bodylanguage model
        - Click on Run Dynamic BodyLanguage Model. Below Properties, under Model File, select the TwoPlates.mod file. 
        - The .mod file is part of the BB_Models folder. On the SwarmLabPC, it is located at:
        ```
        "C:\Users\Public\Documents\Vicon\BB_Models\"
        ```
        - This file can be modifined accordingly to change the position offset of the reference markers. With the current configuation, they create an offset in the Z and Y axis wrt the primary axis marker (more below).
    - Kinematic Fit
    - Static Skeleton Calibration
    Right click on each one and Run. They should all check off.

12. Make sure your subject is saved

13. You will now see that one of the reference marker TS2 will be negatively offset along the z-axis (directly below)from the primary TS1 marker. And reference marker TS3 will be offset along positive y from TS1

So thus there are the 2 reference markers as an additional to the actual subject markers. These markers will allow us to create the axes of the subject so that they are aligned with the world. 

14. Hold ctrl and select all markers on TS (2 Reference + Original subject markers, execpt for the outlier marker if you encouter one). Right click on a marker and select Unlabel Trajectories.

15. Save

16. At this point, you should still be offline.

17. Now create another blank subject. This will be the actual subject so name it accordingly.

18. At this point you should still be offline. Now uncheck TS under Subjects on the right side bar. You should no longer see the lines joining the markers, but only the markers in white.

19. Under the Subject Preparation tool, navigate to Create Segments under Labelling Template Builder and name the segement as you wish and click Create.

20. Now follow the marker selection order **carefully**:
    - Firstly, select the centre marker as the primary marker (as done previously)
    - Now select FS2, which is directly below the primary marker. Z axis will now be upwards (it is counterintuitive that clicking below directs it upwards, but that's how it works)
    - Now select FS3. This direction will define the y-axis. There is no counterintuition here. The direction of FS3 from FS2 will define the y-axis.
    - The x-axis will be defined by the right hand rule -> index = x, middle = y, thumb = z
    - Finally, select the rest of the markers in any order and Click create

21. Save your subject

22. Hold ctrl and select the two reference markers, FS2 and FS3. Right click on a marker and select Detach Markers.

23. Save your subject and go live

You should now be able to your subject with its axes aligned with the world frame. 


Check the video [here](https://drive.google.com/file/d/1cIk3GBsdqz76yD69lcBr7tmNHiTkSdPj/view?usp=sharing) for visual guidance on how to create a subject. 
Check images [1](https://drive.google.com/file/d/16sknleSsSOKMFwMJvKXXNW_eUfVoFZmU/view?usp=sharing) [2](https://drive.google.com/file/d/1Z73omh0kiFJANDx-VxTlwNBuoQz2J0Jt/view?usp=sharing) and [3](https://drive.google.com/file/d/1YQ6YIBG5omY2kNgpnkSk4dAZy6E5Ieya/view?usp=sharing) to see the reference marker placement. Marker along +y (image 2) and along +x (image 3).
