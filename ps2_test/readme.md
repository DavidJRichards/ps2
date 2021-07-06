# PS/2 interface with test project ps2_test

## Features

  * EBAZ4205 target
  * Power on reset
  * Status LED updates
  * ASCII decode
  * Caps lock, num lock, scroll lock
  * Shift, Control
  * UK or US keyboard mapping
  * VT52 cursor codes
  * Debug LED outputs
  * Debug Switch inputs
  * Debug reset button
  * Serial test project
  * Uses 33.333 MHz PL clock
  * 38400 bps serial

Work in progress, initial version

## Doxygen

[DoxygenFilterSystemVerilog](https://github.com/DavidJRichards/DoxygenFilterSystemVerilog) has been used to do automatically create a document to describe the verilog used in this project. running doxygen is the ps2_test directory will create a new html directiry with the output files, open index.html to view the result.

requires: doxygen, graphviz, in addition to perl and DoxygenFilterSystemVerilog
optional: dozygui for doxywizard

