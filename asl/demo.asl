/**
 * Demo BDI program
 * This example BDI program performs the action
 * 'do(action)' whenever it receives a perception 
 * of the format 'time(1234)'.
 * @author	Patrick Gavigan
 * @date	6 December 2019
 */

+position(false,true,false) <- drive(forward).

+position(true,true,false) <- drive(left).

+position(true,false,false) <- drive(left).

+position(false,true,true) <- drive(right).

+position(false,false,true) <- drive(right).

+position(false,false,false) <- drive(stop).

+position(true,true,true) <- drive(stop).
