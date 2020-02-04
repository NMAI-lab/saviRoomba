/**
 * Demo BDI program
 * This example BDI program performs the action
 * 'do(action)' whenever it receives a perception 
 * of the format 'bumper(_,_), where the underscores
 * are boolean data'.
 * @author	Patrick Gavigan
 * @date	4 February 2020
 */

+bumper(True,_) <- lights(True).

+bumper(_,True) <- lights(True).

+bumper(False,False) <- lights(False).
