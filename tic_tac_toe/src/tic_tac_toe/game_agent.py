#!/usr/bin/env python

# Tac Toe
# X = Red/1, O = Blue/2 " " = Blank space/0
import random


def makeMove(board, letter, move):
    board[move] = letter


def isWinner(bo, le):
    # Given a board and a player s letter, this function returns True if that player has won.
    # We use bo instead of board and le instead of letter so we don t have to type as much.
    return ((bo[6] == le and bo[7] == le and bo[8] == le) or  # across the top
            (bo[3] == le and bo[4] == le and bo[5] == le) or  # across the middle
            (bo[0] == le and bo[1] == le and bo[2] == le) or  # across the bottom
            (bo[6] == le and bo[3] == le and bo[0] == le) or  # down the left side
            (bo[7] == le and bo[4] == le and bo[1] == le) or  # down the middle
            (bo[8] == le and bo[5] == le and bo[2] == le) or  # down the right side
            (bo[6] == le and bo[4] == le and bo[2] == le) or  # diagonal
            (bo[8] == le and bo[4] == le and bo[0] == le))  # diagonal


def getBoardCopy(board):
    # Make a duplicate of the board list and return it the duplicate.
    dupeBoard = []
    for i in board:
        dupeBoard.append(i)
    return dupeBoard


def isSpaceFree(board, move):
    # Return true if the passed move is free on the passed board.
    return board[move] == 0


def chooseRandomMoveFromList(board, movesList):
    # Returns a valid move from the passed list on the passed board.
    # Returns None if there is no valid move.
    possibleMoves = []
    for i in movesList:
        if isSpaceFree(board, i):
            possibleMoves.append(i)

    if len(possibleMoves) != 0:
        return random.choice(possibleMoves)
    else:
        return None


def getComputerMove(board, computerLetter, playerLetter):
    # Here is our algorithm for our Tic Tac Toe AI:
    # First, check if we can win in the next move
    for i in range(0, 9):
        copy = getBoardCopy(board)
        if isSpaceFree(copy, i):
            makeMove(copy, computerLetter, i)
            if isWinner(copy, computerLetter):
                return i
    # Check if the player could win on their next move, and block them.
    for i in range(0, 9):
        copy = getBoardCopy(board)
        if isSpaceFree(copy, i):
            makeMove(copy, playerLetter, i)
            if isWinner(copy, playerLetter):
                return i
    # Try to take one of the corners, if they are free.
    move = chooseRandomMoveFromList(board, [0, 2, 6, 8])
    if move != None:
        return move
    # Try to take the center, if it is free.
    if isSpaceFree(board, 4):
        return 4
    # Move on one of the sides.
    return chooseRandomMoveFromList(board, [1, 3, 5, 7])


def isBoardFull(board):
    # Return True if every space on the board has been taken. Otherwise return False.
    for i in range(0, 9):
        if isSpaceFree(board, i):
            return False
    return True


def getMove(theBoard, computerLetter):
    # Given a board and the computer's letter, determine where to move and return that move.
    if computerLetter == 1:
        playerLetter = 2
    else:
        playerLetter = 1
    if isWinner(theBoard, playerLetter):
        print('The player has won!')
        return -1
    elif isBoardFull(theBoard):
        print('The game is a tie!')
        return -1
    move = getComputerMove(theBoard, computerLetter, playerLetter)
    return move
