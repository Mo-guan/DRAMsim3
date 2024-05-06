#ifndef __CPU_H
#define __CPU_H

#include <fstream>
#include <functional>
#include <random>
#include <string>
#include "memory_system.h"

namespace dramsim3 {

class CPU {
   public:
    CPU(const std::string& config_file, const std::string& output_dir)
        : memory_system_(
              config_file, output_dir,
              std::bind(&CPU::ReadCallBack, this, std::placeholders::_1),
              std::bind(&CPU::WriteCallBack, this, std::placeholders::_1)),
          clk_(0) {}
    virtual void ClockTick() = 0;
    void ReadCallBack(uint64_t addr) { return; }
    void WriteCallBack(uint64_t addr) { return; }
    void PrintStats() { memory_system_.PrintStats(); }
    virtual bool finished() { return false; }

   protected:
    MemorySystem memory_system_;
    uint64_t clk_;
};

class RandomCPU : public CPU {
   public:
    using CPU::CPU;
    void ClockTick() override;

   private:
    uint64_t last_addr_;
    bool last_write_ = false;
    std::mt19937_64 gen;
    bool get_next_ = true;
};

class StreamCPU : public CPU {
   public:
    using CPU::CPU;
    void ClockTick() override;

   private:
    static const int num_stream = 3;
    // uint64_t addr_a_, addr_b_, addr_c_, offset_ = 0;
    uint64_t offset_ = 0;
    uint64_t addrs[num_stream];
    std::mt19937_64 gen;
    // bool inserted_a_ = false;
    // bool inserted_b_ = false;
    // bool inserted_c_ = false;
    bool inserted[num_stream] = {false};
    const uint64_t array_size_ = 2 << 20;  // elements in array
    const int stride_ = 64;                // stride in bytes
};

class TraceBasedCPU : public CPU {
   public:
    TraceBasedCPU(const std::string& config_file, const std::string& output_dir,
                  const std::string& trace_file);
    ~TraceBasedCPU() { trace_file_.close(); }
    void ClockTick() override;

   private:
    std::ifstream trace_file_;
    Transaction trans_;
    bool get_next_ = true;
};

class TraceInputStream {
   private:
    std::ifstream file;
    uint64_t expectRecordId;

   public:
    TraceInputStream(const std::string& traceFileName)
        : file(traceFileName.c_str(), std::ios::in | std::ios::binary) {
        reset();
    }

    bool is_open() const { return file.is_open(); }

    bool checkMagic() {
        const uint32_t size = 8;
        char magic[8];
        file.read(reinterpret_cast<char*>(&magic), size);
        return strcmp(magic, "BINFILE") == 0;
    }

    void reset() {
        file.clear();
        file.seekg(0);
        bool correctMagic = checkMagic();
        assert(correctMagic);
        expectRecordId = 0;
    }

    bool parseRecord(MemReq* record, bool* eof) {
        *eof = file.peek() == EOF;
        if (*eof) return false;
        const uint32_t size = 8;
        file.read(reinterpret_cast<char*>(&record->id), size);
        file.read(reinterpret_cast<char*>(&record->addr), size);
        file.read(reinterpret_cast<char*>(&record->type), size);
        file.read(reinterpret_cast<char*>(&record->delay), size);
        // record->delay = 0;
        file.read(reinterpret_cast<char*>(&record->size), size);
        uint64_t depCount;
        file.read(reinterpret_cast<char*>(&depCount), size);
        record->deps.resize(depCount);
        for (size_t i = 0; i < depCount; i++) {
            file.read(reinterpret_cast<char*>(&record->deps[i]), size);
        }
        record->ids.resize(ceil(record->size / 64.0));
        for (size_t i = 0; i < record->ids.size(); i++) {
            record->ids[i] = i;
        }
        // printf(
        //     "id: %ld, addr: %lx, type: %d, delay: %d, size: %d, "
        //     "minIssuse: %d, ids: %d\n",
        //     record->id, record->addr, record->type, record->delay,
        //     record->size, record->min_issue_cycle, record->ids.size());
        return true;
    }

    bool next(MemReq* record, bool* eof) {
        bool good = parseRecord(record, eof);

        if (good) {
            // Check continuous ID.
            if (record->id != expectRecordId) {
                printf(
                    "OpRecord id is not continuous, should be 0x%lx but 0x%lx "
                    "encountered.",
                    expectRecordId, record->id);
                exit(-1);
            }
            expectRecordId++;
        }

        return good;
    }
};

class RamSimCPU : public CPU {
   public:
    RamSimCPU(const std::string& config_file, const std::string& output_dir,
              const std::string& trace_file, int cpu_clock_ration,
              int mem_clock_ratio);
    ~RamSimCPU() {}
    void ClockTick() override;
    bool finished() override;
    bool depSolved(MemReq* record) {
        for (auto& dep : record->deps) {
            for (auto& req : pendReq) {
                if (req.id == dep) {
                    return false;
                }
            }
        }
        return true;
    }

    bool finish = false;

   private:
    TraceInputStream* trace_file_ = nullptr;
    Transaction trans_;
    bool get_next_ = true;
    int cpu_clock_ratio_;
    int mem_clock_ratio_;
    std::vector<MemReq> pendReq;
    uint32_t maxPendEntry = 256;
    bool all_trace_read = false;
};

}  // namespace dramsim3
#endif
