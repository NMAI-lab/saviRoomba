
/**
 * Definition of the map
 * The map is the shape of the letter 'T'.
 */
 
 /**
 * Definition of the map
 * The map is the shape of the letter 'T'.
 * 
 * d----b---c
 * 		|
 * 		|
 * 		|
 * 		a
 */

locationName(a,[1,1]).
locationName(b,[1,0]).
locationName(c,[2,0]).
locationName(d,[0,0]).

// Possible map transitions.
// possible(StartingPosition, PossibleNewPosition)
possible(a,b).
possible(b,a).

possible(b,c).
possible(c,b).

possible(b,d).
possible(d,b).
