#include "behaviour/Behaviour.h"

// wom::Behaviour
wom::Behaviour::wom::Behaviour(std::string name, units::time::second_t period)
    : _bhvr_name(name), _bhvr_period(period), _bhvr_state(wom::BehaviourState::INITIALISED) {}
wom::Behaviour::~wom::Behaviour() {
  if (!IsFinished()) Interrupt();
}

std::string wom::Behaviour::GetName() const {
  return _bhvr_name;
}

void wom::Behaviour::SetPeriod(units::time::second_t Period) {
  _bhvr_period = Period;
}

units::time::second_t wom::Behaviour::GetPeriod() const {
  return _bhvr_period;
}

units::time::second_t wom::Behaviour::GetRunTime() const {
  return _bhvr_timer;
}

void wom::Behaviour::Controls(wom::HasBehaviour *sys) {
  if (sys != nullptr) _bhvr_controls.insert(sys);
}

void wom::Behaviour::Inherit(wom::Behaviour &bhvr) {
  for (auto c : bhvr.GetControlled()) Controls(c);
}

wom::Behaviour::ptr wom::Behaviour::WithTimeout(units::time::second_t timeout) {
  _bhvr_timeout = timeout;
  return shared_from_this();
}

wpi::SmallPtrSetImpl<wom::HasBehaviour *> &wom::Behaviour::GetControlled() {
  return _bhvr_controls;
}

wom::BehaviourState wom::Behaviour::GetBehaviourState() const {
  return _bhvr_state;
}

void wom::Behaviour::Interrupt() {
  Stop(wom::BehaviourState::INTERRUPTED);
}

void wom::Behaviour::SetDone() {
  Stop(wom::BehaviourState::DONE);
}

bool wom::Behaviour::Tick() {
  if (_bhvr_state == wom::BehaviourState::INITIALISED) {
    _bhvr_time  = frc::RobotController::GetFPGATime();
    _bhvr_state = wom::BehaviourState::RUNNING;
    _bhvr_timer = 0_s;

    OnStart();
  }

  if (_bhvr_state == wom::BehaviourState::RUNNING) {
    uint64_t now = frc::RobotController::GetFPGATime();
    auto     dt  = static_cast<double>(now - _bhvr_time) / 1000000 * 1_s;
    _bhvr_time   = now;
    _bhvr_timer += dt;

    if (dt > 2 * _bhvr_period) {
      std::cerr << "wom::Behaviour missed deadline. Reduce Period. Dt=" << dt.value()
                << " Dt(deadline)=" << (2 * _bhvr_period).value() << ". Bhvr: " << GetName() << std::endl;
    }

    if (_bhvr_timeout.value() > 0 && _bhvr_timer > _bhvr_timeout)
      Stop(wom::BehaviourState::TIMED_OUT);
    else
      OnTick(dt);
  }

  return IsFinished();
}

bool wom::Behaviour::IsRunning() const {
  return _bhvr_state == wom::BehaviourState::RUNNING;
}

bool wom::Behaviour::IsFinished() const {
  return _bhvr_state != wom::BehaviourState::INITIALISED && _bhvr_state != wom::BehaviourState::RUNNING;
}

void wom::Behaviour::Stop(wom::BehaviourState new_state) {
  if (_bhvr_state.exchange(new_state) == wom::BehaviourState::RUNNING) OnStop();
}

wom::Behaviour::ptr wom::Behaviour::Until(wom::Behaviour::ptr other) {
  // return shared_from_this() | other;
  auto conc = make<ConcurrentBehaviour>(ConcurrentBehaviourReducer::FIRST);
  conc->Add(other);
  conc->Add(shared_from_this());
  return conc;
}

// Sequential wom::Behaviour
void SequentialBehaviour::Add(ptr next) {
  _queue.push_back(next);
  Inherit(*next);
}

std::string SequentialBehaviour::GetName() const {
  return _queue.front()->GetName();
}

void SequentialBehaviour::OnTick(units::time::second_t dt) {
  if (!_queue.empty()) {
    SetPeriod(_queue.front()->GetPeriod());
    _queue.front()->Tick();
    if (_queue.front()->IsFinished()) {
      _queue.pop_front();
      if (_queue.empty())
        SetDone();
      else
        _queue.front()->Tick();
    }
  } else {
    SetDone();
  }
}

void SequentialBehaviour::OnStop() {
  if (GetBehaviourState() != wom::BehaviourState::DONE) {
    while (!_queue.empty()) {
      _queue.front()->Interrupt();
      _queue.pop_front();
    }
  }
}

// ConcurrentBehaviour
ConcurrentBehaviour::ConcurrentBehaviour(ConcurrentBehaviourReducer reducer)
    : wom::Behaviour(), _reducer(reducer) {}

void ConcurrentBehaviour::Add(wom::Behaviour::ptr behaviour) {
  for (auto c : behaviour->GetControlled()) {
    auto &controls = GetControlled();
    if (controls.find(c) != controls.end()) {
      throw DuplicateControlException(
          "Cannot run behaviours with the same controlled system concurrently "
          "(duplicate in: " +
          behaviour->GetName() + ")");
    }
    Controls(c);
  }

  _children.push_back(behaviour);
  _children_finished.emplace_back(false);
}

std::string ConcurrentBehaviour::GetName() const {
  std::string msg = (_reducer == ConcurrentBehaviourReducer::ALL ? "ALL { " : "RACE {");
  for (auto b : _children) msg += b->GetName() + ", ";
  msg += "}";
  return msg;
}

void ConcurrentBehaviour::OnStart() {
  for (size_t i = 0; i < _children.size(); i++) {
    auto b = _children[i];

    _threads.emplace_back([i, b, this]() {
      while (!b->IsFinished() && !IsFinished()) {
        using namespace std::chrono_literals;

        b->Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds((int64_t)(b->GetPeriod().value() * 1000)));
      }

      if (IsFinished() && !b->IsFinished()) b->Interrupt();

      {
        std::lock_guard lk(_children_finished_mtx);
        _children_finished[i] = true;
      }
    });
  }
}

void ConcurrentBehaviour::OnTick(units::time::second_t dt) {
  bool ok = _reducer == ConcurrentBehaviourReducer::ALL ? true : false;

  {
    std::lock_guard lk(_children_finished_mtx);

    if (_reducer == ConcurrentBehaviourReducer::FIRST) {
      ok = _children_finished[0];
    } else {
      for (bool fin : _children_finished) {
        if (_reducer == ConcurrentBehaviourReducer::ALL) {
          ok = ok && fin;
        } else if (_reducer == ConcurrentBehaviourReducer::ANY) {
          ok = ok || fin;
        }
      }
    }
  }

  if (ok) SetDone();
}

void ConcurrentBehaviour::OnStop() {
  for (auto &t : _threads) {
    t.join();
  }
}

// If
If::If(std::function<bool()> condition) : _condition(condition) {}
If::If(bool v) : _condition([v]() { return v; }) {}

std::shared_ptr<If> If::Then(wom::Behaviour::ptr b) {
  _then = b;
  Inherit(*b);
  return std::reinterpret_pointer_cast<If>(shared_from_this());
}

std::shared_ptr<If> If::Else(wom::Behaviour::ptr b) {
  _else = b;
  Inherit(*b);
  return std::reinterpret_pointer_cast<If>(shared_from_this());
}

void If::OnStart() {
  _value = _condition();
}

void If::OnTick(units::time::second_t dt) {
  wom::Behaviour::ptr _active = _value ? _then : _else;
  if (_active) _active->Tick();
  if (!_active || _active->IsFinished()) SetDone();

  if (IsFinished() && _active && !_active->IsFinished()) _active->Interrupt();
}

// WaitFor
WaitFor::WaitFor(std::function<bool()> predicate) : _predicate(predicate) {}
void WaitFor::OnTick(units::time::second_t dt) {
  if (_predicate()) SetDone();
}

// WaitTime
WaitTime::WaitTime(units::time::second_t time) : WaitTime([time]() { return time; }) {}
WaitTime::WaitTime(std::function<units::time::second_t()> time_fn) : _time_fn(time_fn) {}

void WaitTime::OnStart() {
  _time = _time_fn();
}

void WaitTime::OnTick(units::time::second_t dt) {
  if (GetRunTime() > _time) SetDone();
}

// Print
Print::Print(std::string message) : _message(message) {}
void Print::OnTick(units::time::second_t dt) {
  std::cout << _message << std::endl;
  SetDone();
}