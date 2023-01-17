# Header files
This folder contains all _header_ files relevant to X-View. The header files 
are grouped and organized in folders based on their usage inside the X-View 
library.

Here follows a description of what type of header files are contained in each
 folder:
 
  * [__datasets__](./datasets) folder contains all header files which 
  describe and allow X-View to use different datasets seamlessly.
  
  * [__features__](./features) folder contains all header files which define 
  a set of _features_ (the world "feature" is to be understood in the same 
  way as it is in "feature detection" or "feature matching") used in X-View.
   
  * [__landmarks__](./landmarks) folder contains all header files related to 
  different _landmark_ types. In particular, since the SLAM problem can be 
  takled in various ways (color-based feature detection, depth-based place 
  recognition etc.) the header files in this folder are associated to 
  different landmark types.
  
  * [__matchers__](./matchers) folder contains all header files related to 
  _matching different features_. In particular, since the SLAM problem can be 
  takled using different _features_, different _feature matchers_ need to be 
  used accordingly.
  
  * [__parameters__](./parameters) folder contains the header files 
  associated to a _parameter handler_ class, which is used to handle 
  parameters used at runtime by X-View.
