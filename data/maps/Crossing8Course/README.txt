OpenDRIVE Example "Crossing8Course.xodr"
========================================

A) Licensing:
-------------

    (c) 2010 VIRES Simulationstechnologie GmbH, Germany

    This example is provided only for the illustration of OpenDRIVE.
    All files remain the property of VIRES Simulationstechnologie GmbH,
    Rosenheim, Germany and MUST NOT be modified, re-used or made part 
    of any commercial product. 
    
    The files may be re-distributed by and to third parties in favor 
    of the originally intended purpose. Only the complete package including
    this license agreement may be re-distributed.

    Unless required by applicable law or agreed to in writing, software
    distributed under this License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

B) Purpose:
-----------

This example shall illustrate the modeling of a standard crossing according
to the OpenDRIVE standard. The crossing is controlled by traffic lights, 
static signs serve as backup when the lights are disabled.

Due to the closed design, traffic may drive on the course endlessly.


B) Files:
-----------

The example consists of the OpenDRIVE description of the road network
and a series of simplified graphical data for the visualization.

These are the most important files:

|----Crossing8Course
|    |----Crossing8Course.xodr........OpenDRIVE description
|    |----Flt
|    |    |----Crossing8Course.flt....OpenFlight master file
|    |----Objects.....................OpenFlight objects
|    |----README.txt..................this file
|    |----TexturePool.................Textures for the OpenFlight files

In order to load the visual database, please perform the following steps:
1) Set your file path to the directories "Flt", "Objects" and "TexturePool"
2) Load the file "Flt/Crossing8Course.flt"


January 19, 2010
Marius Dupuis
VIRES Simulationstechnologie GmbH
