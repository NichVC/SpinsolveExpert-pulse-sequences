# SpinsolveExpert-pulse-sequences

I have not been able to find any repositories sharing pulse-sequences designed for the Spinsolve Expert software. I'll be sharing my own selective pulse implementation here - feel free to make a pull request if you have any to share.

## Selective sinc-cos based pulse used for hyperpolarized 13C time-series
<img src="Selective-sinc-cos/Pulse_illustration.png" width=65% height=65%>

Sequences implemented in SpinsolveExpert 1.41.10. The 4 pulse programs can be found in the ”Custom Pulse Programs” folder, where the SpinsolveExpert software is loading them from. ”Carbon_SincCos” is a single pulse and acquire. The ”_AmpSweep” are used for sweeping amplitudes of the pulse and the ”_CenterSweep” is used for sweeping over different center frequencies. The ” _TIMESERIES_dec” is the actual experiment used for 13C time-series with 1H decoupling.

Place the pulse programs in a folder like C:\Users\XXXX\Applications\Custom Pulse Programs

In Spinsolve Expert add the folder with the custom pulse programs and recompile them.

Christensen NV, Laustsen C, Bertelsen LB. Differentiating leukemia subtypes based on metabolic signatures using hyperpolarized 13C NMR. NMR in Biomedicine. 2024;e5264. [doi:10.1002/nbm.5264](https://doi.org/10.1002/nbm.5264)
