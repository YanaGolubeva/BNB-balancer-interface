/*
 * File:   sched.hpp
 * Author: posypkin
 *
 * Scheduler
 *
 * Created on September 15, 2011, 4:15 AM
 */

#ifndef SCHED_HPP
#define	SCHED_HPP

#include <limits>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "bnbscheduler.hpp"
#include "sstrat.hpp"
#include <chaiscript/chaiscript.hpp>
#include <string>

class Sched : public BNBScheduler {
public:
    /**
     * Constructor
     * @param balance_params parameters for balancer
     * @param chaiIniFileName name of file with chai initialization
     * @param chaiSchedFileName name of file with chai scheduler
     */
    Sched(std::vector<int> &balance_params, std::string chaiIniFileName, std::string chaiSchedFileName) {
        mChaiIniFileName = chaiIniFileName;
        mChaiSchedFileName = chaiSchedFileName;
        chai.add(chaiscript::var(&balance_params), "balance_params");
        chai.add(chaiscript::bootstrap::standard_library::vector_type<std::vector<int>>("IntVector"));
        chai.add(chaiscript::bootstrap::standard_library::vector_type<std::vector<long long>>("longLongVector"));
        chai.add(chaiscript::fun(&Sched::M_BNB_ASSERT),"BNB_ASSERT");
        chai.eval_file(mChaiIniFileName);
        mState = 1;
        mMasterSearchStrategy = SearchStrategies::WFS;
        mSlaveSearchStrategy = SearchStrategies::DFS;
        chai.add(chaiscript::var(mMasterSearchStrategy), "mMasterSearchStrategy");
        chai.add(chaiscript::var(mSlaveSearchStrategy), "mSlaveSearchStrategy");

        int event_error = BNBScheduler::Events::ERROR;
        chai.add(chaiscript::var(event_error), "EVENT_ERROR");
        int event_dummy_event = BNBScheduler::Events::DUMMY_EVENT;
        chai.add(chaiscript::var(event_dummy_event), "EVENT_DUMMY_EVENT");
        int event_start = BNBScheduler::Events::START;
        chai.add(chaiscript::var(event_start), "EVENT_START");
        int event_done = BNBScheduler::Events::DONE;
        chai.add(chaiscript::var(event_done), "EVENT_DONE");
        int event_sent = BNBScheduler::Events::SENT;
        chai.add(chaiscript::var(event_sent), "EVENT_SENT");
        int event_nothing_to_send = BNBScheduler::Events::NOTHING_TO_SEND;
        chai.add(chaiscript::var(event_nothing_to_send), "EVENT_NOTHING_TO_SEND");
        int event_data_arrived = BNBScheduler::Events::DATA_ARRIVED;
        chai.add(chaiscript::var(event_data_arrived), "EVENT_DATA_ARRIVED");
        int event_command_arrived = BNBScheduler::Events::COMMAND_ARRIVED;
        chai.add(chaiscript::var(event_command_arrived), "EVENT_COMMAND_ARRIVED");
        int event_search_strategy_set = BNBScheduler::Events::SEARCH_STRATEGY_SET;
        chai.add(chaiscript::var(event_search_strategy_set), "EVENT_SEARCH_STRATEGY_SET");
        int event_heuristic_set = BNBScheduler::Events::HEURISTIC_SET;
        chai.add(chaiscript::var(event_heuristic_set), "EVENT_HEURISTIC_SET");
        int event_bounding_method_set = BNBScheduler::Events::BOUNDING_METHOD_SET;
        chai.add(chaiscript::var(event_bounding_method_set), "EVENT_BOUNDING_METHOD_SET");
        int action_dummy_action = BNBScheduler::Actions::DUMMY_ACTION;
        chai.add(chaiscript::var(action_dummy_action), "ACTION_DUMMY_ACTION");
        int action_send_sub = BNBScheduler::Actions::SEND_SUB;
        chai.add(chaiscript::var(action_send_sub), "ACTION_SEND_SUB");
        int action_send_records = BNBScheduler::Actions::SEND_RECORDS;
        chai.add(chaiscript::var(action_send_records), "ACTION_SEND_RECORDS");
        int action_send_sub_and_records = BNBScheduler::Actions::SEND_SUB_AND_RECORDS;
        chai.add(chaiscript::var(action_send_sub_and_records), "ACTION_SEND_SUB_AND_RECORDS");
        int action_send_command = BNBScheduler::Actions::SEND_COMMAND;
        chai.add(chaiscript::var(action_send_command), "ACTION_SEND_COMMAND");
        int action_recv = BNBScheduler::Actions::RECV;
        chai.add(chaiscript::var(action_recv), "ACTION_RECV");
        int action_set_search_strategy = BNBScheduler::Actions::SET_SEARCH_STRATEGY;
        chai.add(chaiscript::var(action_set_search_strategy), "ACTION_SET_SEARCH_STRATEGY");
        int action_set_heuristic = BNBScheduler::Actions::SET_HEURISTIC;
        chai.add(chaiscript::var(action_set_heuristic), "ACTION_SET_HEURISTIC");
        int action_set_bounding_method = BNBScheduler::Actions::SET_BOUNDING_METHOD;
        chai.add(chaiscript::var(action_set_bounding_method), "ACTION_SET_BOUNDING_METHOD");
        int action_solve = BNBScheduler::Actions::SOLVE;
        chai.add(chaiscript::var(action_solve), "ACTION_SOLVE");
        int action_exit = BNBScheduler::Actions::EXIT;
        chai.add(chaiscript::var(action_exit), "ACTION_EXIT");

        long long max_long = std::numeric_limits<long long>::max();
		chai.add(chaiscript::var(max_long), "max_long");
    }

    void setSearchStrategies(int master, int slave) {
        mMasterSearchStrategy = master;
        mSlaveSearchStrategy = slave;
    }

    static void M_BNB_ASSERT(int i){
        if((i) == 0){
            fprintf(stderr, "ASSERTION FALIED at %s:%d\n", __FILE__, __LINE__);
            fflush(stderr);
            exit(-1);
        }
    }

    void action(const Event& event, const SolverInfo& info, Action& action) {
        if (mRank == 0 && mState == 1) {
            for (int i = 1; i < mSize; i++) {
                mFreeProcs.push_back(i);
            }
        }

        chai.add(chaiscript::var(mRank), "mRank");
        chai.add(chaiscript::var(mSize), "mSize");
        chai.add(chaiscript::var(info.mNSub), "info_mNSub");

        chai.add(chaiscript::var(std::ref(mFreeProcs)), "mFreeProcs");
        chai.add(chaiscript::var(std::ref(event.mCode)), "event_mCode");
        chai.add(chaiscript::var(std::ref(action.mCode)), "action_mCode");

        std::vector<long long> EventmArgs;
        std::vector<long long> ActionmArgs;
        for(int i = 0; i < MAX_ARGS; i ++) {
                EventmArgs.push_back(event.mArgs[i]);
                ActionmArgs.push_back(action.mArgs[i]);
        }
        chai.add(chaiscript::var(std::ref(EventmArgs)), "event_mArgs");
        chai.add(chaiscript::var(std::ref(ActionmArgs)), "action_mArgs");
        chai.add(chaiscript::var(std::ref(mState)), "mState");

        chai.eval_file(mChaiSchedFileName);

        for(int i = 0; i < MAX_ARGS; i ++) {
                action.mArgs[i] = ActionmArgs[i];
        }
    }

    chaiscript::ChaiScript chai;
    int mMasterSearchStrategy;
    int mSlaveSearchStrategy;
    std::vector<int> mFreeProcs;
    //переменная для хранения статуса, полученного в результате отрабатывания скрипта.
    int mState;
    std::string mChaiIniFileName;
    std::string mChaiSchedFileName;
};



#endif	/* SCHED_HPP */

