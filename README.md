This is the source code for our paper: "Viewport Prediction for Live 360-Degree Mobile Video Streaming Using User-Content Hybrid Motion Tracking". This paper is going to be published in IWMUT. Please direct your questions and comments to hwselunl@gmail.com.

# LiveMotion

## hierarchy of this reporsitory:

LiveMotion

    |

    |--------Paper 

    |          |-------"Viewport Prediction for Live 360-Degree Mobile Video Streaming Using User-Content Hybrid Motion Tracking.pdf"

    |        our paper for this project.
    
    |
    
    |--------source code
    
    |           |-----"main_RU.cpp"          Testing the algorithm on the public dataset
    
    |           |-----"main_unl.cpp"         Testing the algorithm on our private dataset
    
    |       the source code for this paper.
    
    |
    |---------testData
    
                |-------Private 4 user   Contains our private dataset
    
                |-------Publicdataset    Contains the public dataset


## Experiment Setup

This work is built with Opencv3.3 (C++) + Visualstudio 17 on Windows 10. 

Please first download and install the Visualstudio 17 for the C++ debugging environment. 

Then, download and intall the [Opencv 3.3] (https://docs.opencv.org/3.3.0/d3/d52/tutorial_windows_install.html)

## Testing & evaluation

Please download the dataset and copy the source data under the same directory of the source code. Run the code and check the output result.

The source code will store the running information (accuracy & bandwidth usage) in the log file (.exl or .txt). You can furthere analyze the performance based on those files.

## the link for the public dataset

The link for the public datset is shown below:
(https://wuchlei-thu.github.io/) 
where you can find more detailed information.