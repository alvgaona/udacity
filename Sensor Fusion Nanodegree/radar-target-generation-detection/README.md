# Radar Target Generation and Detection

A project from Sensor Fusion Nanodegree at Udacity.

# Rubric points met

## FMCW Waveform Design

### Criteria

Using the given system requirements, design a FMCW waveform. Find its bandwidth, chirp time and slope of the chirp.

### Meets Specification

For given system requirements the calculated slope should be around `2e13`.

Implementation can be found in [here][Slope].

## Simulation Loop

### Criteria

Simulate Target movement and calculate the beat or mixed signal for every timestamp.

### Meets Specification

A beat signal should be generated such that once range FFT implemented, it gives the correct range i.e the initial position of target assigned with an error margin of +/- 10 meters.

Implementation can be found in [here][Simulation Loop].

## Range FFT (1st FFT)

### Criteria

Implement the Range FFT on the Beat or Mixed Signal and plot the result.

### Meets Specification

A correct implementation should generate a peak at the correct range, i.e the initial position of target assigned with an error margin of +/- 10 meters.

Implementation can be found in [here][Range measurement].

![Range First FFT]

![2D FFT]

## 2D CFAR

### Criteria

Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.

### Meets Specification

The 2D CFAR processing should be able to suppress the noise and separate the target signal. 
The output should match the image shared in walkthrough.

Implementation can be found in [here][2D CFAR].

![CFAR Output]

The CFAR implementation is described below.

1. Determine the number of training cells for each dimension, both range and doppler.
Also, pick the number of guard cells for each dimension.

```
Tr = 10 // Number of training cells (range dimension)
Td = 8  // Number of training cells (doppler dimension)
Gr = 4  // Number of guard cells (range dimension)
Gd = 4  // Number of guard cells (doppler dimension)
```

2. Slide the Cell Under Test (CUT) across the complete cell matrix.

3. Select the grid that includes the training, guard and test cells.

```
gridSize = [ 2Tr+2Gr+1, 2Td+2Gd+1 ]
```

4. The total number of cells in the guard region and cell under test. 

```
totalGuardCells = (2Gr+1) x (2Gd+1)
```

This leaves the number of training cells to

```
totalTrainingCells = (2Tr+2Gr+1) x (2Td+2Gd+1) - totalGuardCells
```

5. Measure and average the noise across all the training cells.
This gives the threshold.

Measure and average noise [loop][Measure Average Noise].

6. Add the offset (if in signal strength in dB) to the threshold to keep the false alarm to the minimum.

Add offset to threshold. 
Follow up this [link][Offset and Threshold].

7. Determine the signal level at the CUT (Cell Under Test).

8. If the CUT signal level is greater than the Threshold, assign a value of 1, else equate it to zero.

9. Since the cell under test is not located at the edges, due to the training cells occupying the edges, suppress the edges to zero.
Any cell value that is neither 1 nor a 0, assign it a zero.

[Slope]: RadarTargetGenerationDetection.m#L25
[Simulation Loop]: RadarTargetGenerationDetection.m#L39
[Range Measurement]: RadarTargetGenerationDetection.m#L63
[2D CFAR]: RadarTargetGenerationDetection.m#L81
[Measure Average Noise]: RadarTargetGenerationDetection.m#L95
[Offset and Threshold]: RadarTargetGenerationDetection.m#L104

[Range First FFT]: images/rangeFirstFFT.png
[2D FFT]: images/2DFFT.png
[CFAR Output]: images/CFAROutput.png