#define N 5
/*****Elevator states*******/
mtype = {idle, running, wait, stopped}
mtype elevator_state;
bit positive_direction;
byte current_stage;


/*******Common Data*******/
bit stage_buttons[N];
bit inner_buttons[N];
bit sensors[N];
bit doors[N];
bit button_open;
bit button_stop;

/*******Caller states*********/
mtype = {inside, outside, gone}
mtype caller_state;
byte caller_departure;
byte caller_destination;
inline is_caller_inside()
{
	caller_state == inside;
}


/******Communication between closer and elevator*****/

/*Closer responses*/
mtype = {open_ack, time_out, waiting, close_ack};
/*Elevator requests*/
mtype = {status_request, open_request, close_request};

chan e2c = [0] of {mtype}
chan c2e = [0] of {mtype}


inline stage_call_waiting()
{
	atomic
	{
		if
		:: stage_buttons[0] -> 
			if
			:: current_stage < 0 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: stage_buttons[1] -> 
			if
			:: current_stage < 1 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: stage_buttons[2] -> 
			if
			:: current_stage < 2 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: stage_buttons[3] -> 
			if
			:: current_stage < 3 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: stage_buttons[4] -> 
			if
			:: current_stage < 4 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		fi;
	}
}


inline inner_call_waiting()
{
	atomic
	{
		if
		:: inner_buttons[0] -> 
			if
			:: current_stage < 0 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: inner_buttons[1] -> 
			if
			:: current_stage < 1 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: inner_buttons[2] -> 
			if
			:: current_stage < 2 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: inner_buttons[3] -> 
			if
			:: current_stage < 3 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		:: inner_buttons[4] -> 
			if
			:: current_stage < 4 -> positive_direction = 1;
			:: else -> positive_direction = 0;
			fi;
		fi;
	}
}

inline special()
{
	button_stop || button_open;
}

inline step(cur, dir)
{
	d_step
	{
		if
		:: dir -> cur++;
		:: else -> cur--;
		fi;
	}
}

inline hasCallFrom(stage)
{
	d_step
	{
		stage_buttons[stage] || inner_buttons[stage] || button_open;
	}
}

inline resetInnerButtons()
{
	d_step
	{
		inner_buttons[0] = 0;
		inner_buttons[1] = 0;
		inner_buttons[2] = 0;
		inner_buttons[3] = 0;
		inner_buttons[4] = 0;
	}
}


active proctype elevator()
{
	current_stage = 0;
	positive_direction = 1;
	button_open = 0;
	button_stop = 0;
	end_Route:
		if
		:: inner_call_waiting();
		:: else ->
			if
			:: inner_call_waiting();
			:: stage_call_waiting();
			:: special();
			fi;
		fi;
		elevator_state = running;
		goto Running;
	end_Stopped:
		resetInnerButtons();
		button_open = 0;
		if
		:: inner_call_waiting();
		:: button_open;
		fi;
		button_stop = 0;
		elevator_state = running;
		goto Running;
	Running:
		if
		:: hasCallFrom(current_stage)-> 
				elevator_state = wait;
				goto Wait;
		:: button_stop ->
			elevator_state = stopped;
			goto end_Stopped;
		:: else -> 
			step(current_stage, positive_direction);
			goto Running;
		fi;
	Wait:
		stage_buttons[current_stage] = 0;
		inner_buttons[current_stage] = 0;
		e2c ! open_request;
		c2e ? open_ack;
	SendStatus:
		e2c ! status_request;
	Receive:
		if
		:: c2e ? waiting ->
			goto SendStatus;
		:: c2e ? time_out -> 
			e2c ! close_request;
			goto Receive;
		:: c2e ? close_ack;
		fi;
		d_step
		{
			elevator_state = idle;
			button_stop = 0;
			button_open = 0;
		};
		goto end_Route;

};


active proctype closer()
{
end_Wait_open:
	e2c ? open_request ->
		doors[current_stage] = 1;
		c2e ! open_ack;
	do
	:: e2c ? status_request ->
		if
		:: !(caller_state == inside) ->
			c2e ! waiting;
		:: c2e ! time_out;
		fi;
	:: e2c ? close_request ->
		sensors[current_stage] = 0;
		if
		:: sensors[current_stage] -> c2e ! waiting;
		:: else -> 
			doors[current_stage] = 0;
			c2e ! close_ack;
			break;
		fi;
	od;
	goto end_Wait_open;
}


inline chooser(ch)
{
	atomic
	{
		if
		:: ch = 0;
		:: ch = 1;
		:: ch = 2;
		:: ch = 3;
		:: ch = 4;
		fi;
	}
}

inline enter(stage)
{
	d_step
	{
		doors[stage];
		sensors[stage] = 1;
		caller_state = inside;
	}
}

inline exit(stage)
{
	d_step
	{
		doors[stage];
		sensors[stage] = 1;
		caller_state = outside;
	}
}

active proctype caller()
{
end_Outside:
	caller_state = outside;
	chooser(caller_departure);
	chooser(caller_destination);
	if
	:: enter(caller_destination) ->
		goto Inside;
	:: else
	fi;
	stage_buttons[caller_departure] = 1;
	if
	:: enter(caller_departure) ->
		goto Inside;
	:: else ->
		caller_state = gone;
		goto end_Outside;
	fi;
Inside:
	do
	:: inner_buttons[caller_destination] = 1;
	:: exit(caller_destination) ->
		goto end_Outside;
	:: true ->
		d_step
		{
			button_open = 1;
			caller_destination = current_stage;
		}
	:: true ->
		chooser(caller_destination);
		button_stop = 1;
	od;
};


#define calls ((stage_buttons[0] || stage_buttons[1] || stage_buttons[2] || stage_buttons[3] || stage_buttons[4]) || (inner_buttons[0] || inner_buttons[1] || inner_buttons[2] || inner_buttons[3] || inner_buttons[4]))
#define Out (caller_state != inside)
#define E_Wait (elevator_state == wait)
#define close(i) ((current_stage != i) -> !doors[i])
#define Inside (caller_state == inside)


ltl cantStop
{
	[](elevator_state == running -> (elevator_state == running) U (elevator_state == wait || elevator_state == stopped))
}



ltl keepWorking
{
	[](calls -> <> (elevator_state != idle))
}



ltl shouldWait
{
	[](Out -> Out U E_Wait)
}


ltl shouldExitWhereWant
{
	[](Inside -> Inside W (Out && (elevator_state == wait) && (current_stage == caller_destination)))
}


ltl globalIn
{
	[](([](caller_state != gone)) -> (<> Inside))
}

ltl allClosed
{
	[](!E_Wait -> (!doors[0] && !doors[1] && !doors[2] && !doors[3] && !doors[4]))
}


ltl noWrongOpens
{
	[](close(0) && close(1) && close(2) && close (3) && close(4))
}

ltl exitPossibility
{
	[](Inside -> [](<> (doors[current_stage] || elevator_state == stopped)))
}

ltl specialWithin
{
	[]((!button_stop && !button_open) -> (!button_stop && !button_open) W (caller_state == inside))
}


