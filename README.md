# SpinsolveExpert-pulse-sequences

I have not been able to find any repositories sharing pulse-sequences designed for the Spinsolve Expert software. I'll be sharing my own selective pulse implementation here -feel free to make a pull request if you have any to share.

## Selective sinc-cos based pulse used for hyperpolarized 13C time-series

Sequences implemented in SpinsolveExpert 1.41.10. The 4 pulse programs can be found in the ”Custom Pulse Programs” folder, where the SpinsolveExpert software is loading them from. ”Carbon_SincCos” is a single pulse and acquire. The ”_AmpSweep” are used for sweeping amplitudes of the pulse and the ”_CenterSweep” is used for sweeping over different center frequencies. The ” _TIMESERIES_dec” is the actual experiment used 13C time-series with 1H decoupling.

Also included are Python scripts to analyze amplitude and center frequency sweep data (data examples in respective folders). Note that [nmrglue](https://github.com/jjhelmus/nmrglue) is required.
