/**
 * Demo BDI program
 * This example BDI program performs the action
 * 'do(action)' whenever it receives a perception 
 * of the format 'bumper(_,_), where the underscores
 * are boolean data'.
 * @author	Patrick Gavigan
 * @date	4 February 2020
 */

+bumper(true,_) <- lights(true).

+bumper(_,true) <- lights(true).

+bumper(false,false) <- lights(false).
