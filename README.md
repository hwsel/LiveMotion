# LiveMotion

This is the project repository for our LiveMotion paper published in Ubicomp'19: 

Xianglong Feng, Viswanathan Swaminathan, Sheng Wei, Viewport Prediction for Live 360-Degree Mobile Video Streaming Using User-Content Hybrid Motion Tracking. Proc. ACM Interact. Mob. Wearable Ubiquitous Technol. 3, 2, Article 43, 22 pages, June 2019. 

The paper can be found [here](https://github.com/hwsel/LiveMotion/blob/master/paper/Viewport_Prediction_for_Live_360_Degree_Mobile_Video_Streaming_Using_User_Content_Hybrid_Motion_Tracking.pdf).

Please direct your questions and comments to hwselunl@gmail.com.


## Reporsitory Hierarchy:

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
    


## Experimental Setup

This project is built with Opencv3.3 (C++) + Visual Studio 17 on Windows 10. 

Please first download and install the Visual Studio 17 for the C++ debugging environment. 

Then, download and intall the [Opencv 3.3] (https://docs.opencv.org/3.3.0/d3/d52/tutorial_windows_install.html)

## Testing & Evaluation

Please download the dataset and copy the source data under the same directory of the source code. Run the code and check the output result.

The source code will store the running information (accuracy & bandwidth usage) in the log file (.exl or .txt). You can furthere analyze the performance based on those files.
