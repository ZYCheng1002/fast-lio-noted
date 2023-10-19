//
// Created by czy on 2023/10/17.
//

#ifndef FAST_LIO_TIMER_H
#define FAST_LIO_TIMER_H
#include <glog/logging.h>
#include <chrono>
#include <map>
/// 统计时间工具
class Timer {
 public:
  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage) {
      func_name_ = name;
      time_usage_in_ms_.emplace_back(time_usage);
    }
    std::string func_name_;
    std::vector<double> time_usage_in_ms_;
  };

  /**
   * 评价并记录函数用时
   * @tparam F
   * @param func
   * @param func_name
   */
  template <class F>
  static void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::steady_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

    if (records_.find(func_name) != records_.end()) {
      records_[func_name].time_usage_in_ms_.emplace_back(time_used);
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  /**
   * 仅记录时间
   */
  static void Evaluate(const std::string& func_name, const double& time_us) {
    auto time_used = time_us * 1000;
    if (records_.find(func_name) != records_.end()) {
      records_[func_name].time_usage_in_ms_.emplace_back(time_used);
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  /// 打印记录的所有耗时
  static void PrintAll();

  /// 写入文件，方便作图分析
  static void DumpIntoFile(const std::string& file_name);

  /// 获取某个函数的平均执行时间
  static double GetMeanTime(const std::string& func_name);

  /// 清理记录
  static void Clear() { records_.clear(); }

 private:
  static std::map<std::string, TimerRecord> records_;
};

#endif  // FAST_LIO_TIMER_H
