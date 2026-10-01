#include "mantle/commands/auto_command.h"
#include "core/utils/formatting.h"

class OrCondition : public Condition {
  public:
    OrCondition(Condition *A, Condition *B) : A(A), B(B) {}
    bool test() override {
        bool a = A->test();
        bool b = B->test();
        return a | b;
    }

  private:
    Condition *A;
    Condition *B;
};

class AndCondition : public Condition {
  public:
    AndCondition(Condition *A, Condition *B) : A(A), B(B) {}
    bool test() override {
        bool a = A->test();
        bool b = B->test();
        return a & b;
    }

  private:
    Condition *A;
    Condition *B;
};
std::string Condition::toString() { return "Condition"; }

Condition *Condition::Or(Condition *b) { return new OrCondition(this, b); }

Condition *Condition::And(Condition *b) { return new AndCondition(this, b); }

bool FunctionCondition::test() { return cond(); }
IfTimePassed::IfTimePassed(double time_s) : time_s(time_s), tmr() {}
bool IfTimePassed::test() { return tmr.value() > time_s; }

InOrder::InOrder(std::queue<AutoCommand *> cmds) : cmds(cmds) {
    timeout_seconds = -1.0; // never timeout unless with_timeout is explicitly called
}
InOrder::InOrder(std::initializer_list<AutoCommand *> cmds) : cmds(cmds) { timeout_seconds = -1.0; }

bool InOrder::run() {
    // outer loop finished
    if (cmds.size() == 0 && current_command == nullptr) {
        return true;
    }
    // retrieve and remove command at the front of the queue
    if (current_command == nullptr) {
        printf("TAKING INORDER: len =  %d\n", cmds.size());
        current_command = cmds.front();
        cmds.pop();
        tmr.reset();
    }

    // run command
    bool cmd_finished = current_command->run();
    if (cmd_finished) {
        printf("InOrder Cmd finished\n");
        current_command = nullptr;
        return false; // continue onto next command
    }

    double seconds = tmr.value();

    bool should_timeout = current_command->timeout_seconds > 0.0;
    bool doTimeout = should_timeout && seconds > current_command->timeout_seconds;
    if (current_command->true_to_end != nullptr) {
        doTimeout = doTimeout || current_command->true_to_end->test();
    }

    // timeout
    if (doTimeout) {
        printf("InOrder timed out\n");
        current_command->on_timeout();
        current_command = nullptr;
        return false;
    }
    return false;
}

std::string InOrder::toString() { return "Running Inorder with length: " + int_to_string(cmds.size()); }

void InOrder::on_timeout() {
    if (current_command != nullptr) {
        current_command->on_timeout();
    }
}

Parallel::Parallel(std::initializer_list<AutoCommand *> cmds)
    : cmds(cmds), finished_flags(cmds.size(), false) {}

bool Parallel::run() {
    bool all_finished = true;
    for (size_t i = 0; i < cmds.size(); ++i) {
        if (!finished_flags[i]) {
            if (cmds[i] && cmds[i]->run()) {
                finished_flags[i] = true;
            } else {
                all_finished = false;
            }
        }
    }
    return all_finished;
}

std::string Parallel::toString() { return double_to_string(cmds.size()) + " commands running in parallel"; }

void Parallel::on_timeout() {
    for (size_t i = 0; i < cmds.size(); ++i) {
        if (!finished_flags[i] && cmds[i] != nullptr) {
            cmds[i]->on_timeout();
        }
    }
}

Branch::Branch(Condition *cond, AutoCommand *false_choice, AutoCommand *true_choice)
    : false_choice(false_choice), true_choice(true_choice), cond(cond), choice(false), chosen(false), tmr() {
    this->timeout_seconds = -1;
}

Branch::~Branch() {
    delete false_choice;
    delete true_choice;
};
bool Branch::run() {
    if (!chosen) {
        choice = cond->test();
        chosen = true;
        tmr.reset();
    }

    double seconds = static_cast<double>(tmr.time()) / 1000.0;
    if (choice == false) {
        if (seconds > false_choice->timeout_seconds && false_choice->timeout_seconds != -1) {
            false_choice->on_timeout();
            chosen = false;
            return true;
        }
        bool finished = false_choice->run();
        if (finished) {
            chosen = false;
            return finished;
        }
    } else {
        if (seconds > true_choice->timeout_seconds && true_choice->timeout_seconds != -1) {
            true_choice->on_timeout();
            chosen = false;
            return true;
        }
        bool finished = true_choice->run();
        if (finished) {
            chosen = false;
            return finished;
        }
    }
    return false;
}

std::string Branch::toString() {
    return "Branch of " + false_choice->toString() + " and " + true_choice->toString() + " depending on " +
           cond->toString();
}
void Branch::on_timeout() {
    if (!chosen) {
        // dont need to do anything
        return;
    }

    if (choice == false) {
        false_choice->on_timeout();
    } else {
        true_choice->on_timeout();
    }
    chosen = false;
}

Async::SpawnerFn Async::s_spawner = nullptr;

void Async::set_spawner(SpawnerFn spawner) {
    s_spawner = spawner;
}

bool Async::run() {
    if (s_spawner) {
        s_spawner(cmd);
    } else if (cmd) {
        cmd->run();
    }
    return true;
}

std::string Async::toString() { return "Async of " + cmd->toString(); }

RepeatUntil::RepeatUntil(InOrder cmds, size_t times) : RepeatUntil(cmds, new TimesTestedCondition(times)) {
    timeout_seconds = -1.0;
}

RepeatUntil::RepeatUntil(InOrder cmds, Condition *cond) : cmds(cmds), working_cmds(new InOrder(cmds)), cond(cond) {
    timeout_seconds = -1.0;
}

bool RepeatUntil::run() {
    bool finished = working_cmds->run();
    if (!finished) {
        // return if we're not done yet
        return false;
    }
    // this run finished

    bool res = cond->test();
    // we should finish
    if (res) {
        return true;
    }
    working_cmds = new InOrder(cmds);

    return false;
}

std::string RepeatUntil::toString() {
    InOrder pHCmds = cmds;
    return "Repeating " + pHCmds.toString() + " until " + true_to_end->toString();
}

void RepeatUntil::on_timeout() { working_cmds->on_timeout(); }
