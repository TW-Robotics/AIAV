#!/usr/bin/env python3

# Dieses Skript implementiert die Policy und Value Iteration sowie Q-Learning zum Lösen eines diskreten Gym Environments.
#
# This code is available under a GPL v3.0 license and comes without
# any explicit or implicit warranty.
#
# (C) Simon Schwaiger 2021 <schwaige@technikum-wien.at>

import numpy as np
import gym
import gym_fhtw3dof
from random import random
import math
from geometry_msgs.msg import Point

class iterations():
    """!@brief Helper Class enabling computing a policy for an agent in the gym environment using value and policy iteration. """
    def __init__(self):
        """ Class constructor """
        self.discount_factor = 0.999
        self.max_iterations = 1000
        self.gamma = 1.0
        self.max_steps = 250
        self.render = True

    def calculate_action_values(self, env, state, V):
        """ Calculate Action Values for a given state. """

        # init action value as zero for each possible action (env.nA)
        action_values = np.zeros(env.nA)
        
        # loop over all possible actions
        for action in range(env.nA):
            # loop over the state transitions and calculate value
            for p, next_s, r, _ in env.P[state][action]:
                action_values[action] += p * (r + (self.discount_factor * V[next_s]))
        
        return action_values

    def update_policy(self, env, policy, V):
        """ Update policy based on a value function. """

        for state in range(env.nS):
            # get action values
            action_values = self.calculate_action_values(env, state, V)

            # store index of the max action value (i.e. the action that results in the state with max value)
            policy[state] = np.argmax(action_values)

        return policy

    def policy_evaluation(self, env, policy, V):
        """ Calculates and returns the value of a policy. """

        # init value of all states as zero
        value = np.zeros(env.nS)

        # loop through all state action pairs
        for state, action in enumerate(policy):
            # loop over the state transitions and calculate value
            for p, next_s, r, _ in env.P[state][action]:
                value[state] += p * (r + (self.discount_factor * V[next_s]))
        
        return value
    
    def run_episode(self, env, policy):
        """ Runs through one episode in the environment. """

        state = env.reset()
        total_reward = 0
        s_idx = 0

        while True:
            # get action from policy
            action = int(policy[state])
            # perform action
            state, reward, done, _ = env.step(action)
            # add to total reward and increment step index
            total_reward += reward * (self.gamma**s_idx)
            s_idx += 1

            # apply action to robot (comment out for use with generic gym environments)
            if self.render: env.render_state(state)
            #if self.render: env.render()

            # if the max number of steps are reached or the agent is done, break and return the total reward
            if s_idx >= self.max_steps:
                total_reward = -1
                break
            elif done:
                break

        return total_reward
    
    def run_n_episodes(self, env, policy, num = 100):
        """ Runs num episodes and reports average total reward. """
        scores = []

        for _ in range(num):
            scores.append(self.run_episode(env, policy))

        return np.mean(scores)
    
    def policy_iteration(self, env):
        """ Performs policy iteration and returns policy. """

        # init value function at zero
        V = np.zeros(env.nS)

        # init random policy
        policy = np.random.randint(0, env.nA, env.nS)
        prev_policy = np.copy(policy)

        for i in range(self.max_iterations):
            # evaluate policy
            V = self.policy_evaluation(env, policy, V)
            # update policy based on value function
            policy = self.update_policy(env, policy, V)
            # check if policy changed over the last 10 iterations and exit loop if it did not
            if i % 10 == 0:
                if (np.all(np.equal(policy, prev_policy))):
                    print("Policy converged at {}. iteration.".format(i+1))
                    break
                prev_policy = np.copy(policy)

        return policy

    def value_iteration(self, env):
        """ Performs value iteration and returns policy """
        
        # init value function at zero
        V = np.zeros(env.nS)

        for i in range(self.max_iterations):

            # store previous value function
            prev_V = np.copy(V)

            for state in range(env.nS):
                
                # calculate state action value for each state and action
                action_val = self.calculate_action_values(env, state, prev_V)

                # store the best action based on state action value
                V[state] = np.max(action_val)
        
            # check if value changed over the last 10 iterations and exit loop if it did not
            if i % 10 == 0:
                if (np.all(np.isclose(V, prev_V))):
                    print("Value converged at {}. iteration.".format(i+1))
                    break
        
        # init policy with zeros
        policy = np.zeros(env.nS)

        # compute policy based on optimal value function
        policy = self.update_policy(env, policy, V)

        return policy


class q_learner():
    """!@brief Helper Class enabling computing a policy for an agent in the gym environment using value and policy iteration. """
    def __init__(self):
        """ Class constructor """
        self.learning_rate = 0.85
        self.discount_factor = 0.99
        self.max_episodes = 2000
        self.max_steps = 250
        self.gamma = 1.0
        self.render = True

    def compute_policy(self, env):
        """ Comuptes and returns a policy for the agent in env. """

        # setup storing of all rewards
        rewards = []
        policy = np.zeros(env.nS)

        # init Q-table with zeros
        Q = np.zeros([env.nS, env.nA])

        for i in range(self.max_episodes):
            state = env.reset()
            done = False
            step = 0
            total_reward = 0
            states = []
            actions = []

            while not done and step <= self.max_steps:
                step += 1

                # choose an action from the Q-table. noise is added for exploration
                action = np.argmax(Q[state,:] + np.random.randn(1,env.nA)*(5.0/(i+1)))

                # perform action
                next_s, reward, done, _ = env.step(action)

                # Add negative reward every step in order to motivate quick task completion
                if reward == 0: reward = -0.001

                # Update Q function
                Q[state,action]= Q[state,action]+self.learning_rate*(reward+self.discount_factor* np.max(Q[next_s,:])-Q[state,action])

                # store step data and increment
                state = next_s
                total_reward += reward
                states.append(state)
                actions.append(action)
                           
                # apply action to robot (comment out for use with generic gym environments)
                if self.render: env.render_state(state)
                #if self.render: env.render()
        
            # store episode reward
            rewards.append(total_reward)

            # if episode was succesful store policy of succesful episode
            if reward >= 1: 
                for s in range(env.nS):
                    policy[s] = np.argmax(Q[s,:])
        
        return policy

    def run_episode(self, env, policy):
        """ Runs through one episode in the environment. """

        state = env.reset()
        total_reward = 0
        s_idx = 0

        while True:
            # get action from policy
            action = int(policy[state])
            # perform action
            state, reward, done, _ = env.step(action)
            # add to total reward and increment step index
            total_reward += reward * (self.gamma**s_idx)
            s_idx += 1

            # apply action to robot (comment out for use with generic gym environments)
            if self.render: env.render_state(state)
            #if self.render: env.render()

            # if the max number of steps are reached or the agent is done, break and return the total reward
            if s_idx >= self.max_steps:
                total_reward = -1
                break
            elif done:
                break

        return total_reward
    
    def run_n_episodes(self, env, policy, num = 100):
        """ Runs num episodes and reports average total reward. """
        scores = []

        for _ in range(num):
            scores.append(self.run_episode(env, policy))

        return np.mean(scores)


def setup_environment():
    """ Sets up the FHTW3DOF gym environment for usage with the SAImon robot for the usecase. """
    env = gym.make('fhtw3dof-v0', Stop_Early=False, Constrain_Workspace=False,
                    GoalX=0.0, GoalY=0.0, GoalZ=0.48,
                    J1Limit=31, J2Limit=31, J3Limit=31, joint_res = 0.1,
                    J1Scale=1, J2Scale=-1, J3Scale=-1, J3Offset=0.5,
                    Debug=True, Publish_Frequency=500, Goal_Max_Distance=0.1)
    assert (env.reachable)
    #set isd to every valid point 
    #TODO may break with constrain workspace = True
    env.isd.fill(0)
    for s in range(env.nS):
        if env.Colliding_States[s] == False: env.isd[s] = 1
        else: env.isd[s] = 0
    env.isd /= env.isd.sum()
    print("{} initial states set".format(np.count_nonzero(env.isd)))
    env.Publish_Frequency = 20
    env.P = env.change_transition_prob(env.P, 0.97)
    return env


if __name__ == "__main__":
    # execute only if run as a script, setup env and compute policy using value and policy iteration
    env = setup_environment()
    #env = gym.make("FrozenLake-v0")

    it = iterations()
    it.render = False

    print("Policy Iteration")
    policy = it.policy_iteration(env)
    print("Evaluating Policy Iteration Policy")
    score = it.run_n_episodes(env, policy, num=100)
    print("Policy Iteration Average Score: {}".format(score))

    print("Value Iteration")
    policy = it.value_iteration(env)
    print("Evaluating Value Iteration Policy")
    score = it.run_n_episodes(env, policy, num=100)
    print("Value Iteration Average Score: {}".format(score))

    ql = q_learner()
    ql.render = False

    print("Q-Learning")
    policy = ql.compute_policy(env)
    print("Evaluating Q-Learning Policy")
    score = ql.run_n_episodes(env, policy, num=100)
    print("Q-Learning Average Score: {}".format(score))


