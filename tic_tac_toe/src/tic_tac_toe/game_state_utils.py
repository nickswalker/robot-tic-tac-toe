RED = 1
BLUE = 2


def state_difference(state, state_prime):
    diff = []
    for i in range(len(state)):
        if state[i] != state_prime[i]:
            diff.append(state_prime[i])
        else:
            diff.append(0)
    return diff

def num_pieces_changed(diff):
    # Count non-zero
    return sum([1 for item in diff if item != 0])

def who_played(state, state_prime):
    if is_valid_transition(state, state_prime):
        diff = state_difference(state, state_prime)
        # What is the type of the one piece that changed?
        changed = [item for item in diff if item != 0]
        return changed[0]

def count_piece_types(state):
    type_counts = {RED: 0, BLUE: 0, 0: 0}
    for item in state:
        type_counts[item] = type_counts[item] + 1
    return type_counts

def count_pieces(state):
    return sum([1 for item in state if item != 0])

def whose_turn_is_it(state, first_piece_type):
    if not is_valid(state):
        return None
    type_counts = count_piece_types(state)
    #second_player = RED if first_piece_type == BLUE else BLUE
    #type_counts[second_player] = type_counts[second_player] + 1
    if type_counts[RED] > type_counts[BLUE]:
        return BLUE
    elif type_counts[RED] < type_counts[BLUE]:
        return RED
    elif type_counts[RED] == type_counts[BLUE]:
        return first_piece_type
    else:
        return None

def is_valid(state):
    type_counts = count_piece_types(state)
    piece_delta = abs(type_counts[RED] - type_counts[BLUE])
    if piece_delta > 1:
        return False
    return True


def is_valid_transition(state, state_prime):
    changed = num_pieces_changed(state_difference(state, state_prime))
    num_new = count_pieces(state_prime) - count_pieces(state)
    return changed == 1 and num_new == 1


def is_two_step_transition(state, state_prime):
    changed = num_pieces_changed(state_difference(state, state_prime))
    num_new = count_pieces(state_prime) - count_pieces(state)
    return changed == 2 and num_new == 2
