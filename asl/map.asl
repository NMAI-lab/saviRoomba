/**
 * Map
 * 
 * 		a
 * 		|
 * 		|
 * d -- b
 *		|
 *		|
 *		c
 */
 
// Possible routes between locations
possible(a,b).
possible(b,a).

possible(b,c).
possible(c,b).

possible(b,d).
possible(d,b).


// Location definitions
locationName(a,[1,1]).
locationName(b,[1,0]).
locationName(c,[1,-1]).
locationName(d,[0,0]).	// Robot has to start here, facing b.

// Successor state
suc(Current,Next,Range,drive)
	:-	possible(Current,Next)
		& locationName(Current,[X1,Y1])
		& locationName(Next,[X2,Y2])
		& range(X1,Y1,X2,Y2,Range).
	
// Heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,Range) 
	:-	locationName(Current,[X1,Y1])
		& locationName(Goal,[X2,Y2])
		& range(X1,Y1,X2,Y2,Range).
					
// Range
range(X1,Y1,X2,Y2,Range)
	:-	Range = math.sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) ).
	
atLocation(Location,0)
	:-	locationName(Location,[X,Y])
		& position(X,Y).
		
// Initial beliefs for position and direction faced.
position(0,0).		// Initial location is [0,0]
direction(e).		// See below

// Directions
// 		n
// 		|
// w -- + -- e
//		|
//		s


