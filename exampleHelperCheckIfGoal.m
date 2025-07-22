function isReached = exampleHelperCheckIfGoal(planner, newState, goalState)
%EXAMPLEHELPERCHECKIFGOAL

    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end

end
