#include "cpu.h"

namespace dramsim3 {

void RandomCPU::ClockTick() {
    // Create random CPU requests at full speed
    // this is useful to exploit the parallelism of a DRAM protocol
    // and is also immune to address mapping and scheduling policies
    memory_system_.ClockTick();
    if (get_next_) {
        last_addr_ = gen();
        last_write_ = (gen() % 3 == 0);
    }
    get_next_ = memory_system_.WillAcceptTransaction(last_addr_, last_write_);
    if (get_next_) {
        memory_system_.AddTransaction(last_addr_, last_write_);
    }
    clk_++;
    return;
}

void StreamCPU::ClockTick() {
    // stream-add, read 2 arrays, add them up to the third array
    // this is a very simple approximate but should be able to produce
    // enough buffer hits

    // moving on to next set of arrays
    memory_system_.ClockTick();
    if (offset_ >= array_size_ || clk_ == 0) {
        // addr_a_ = gen();
        // addr_b_ = gen();
        // addr_c_ = gen();
        for (int i = 0; i < num_stream; i++) {
            addrs[i] = gen();
        }
        offset_ = 0;
    }

    // if (!inserted_a_ &&
    //     memory_system_.WillAcceptTransaction(addr_a_ + offset_, false)) {
    //     memory_system_.AddTransaction(addr_a_ + offset_, false);
    //     inserted_a_ = true;
    // }
    // if (!inserted_b_ &&
    //     memory_system_.WillAcceptTransaction(addr_b_ + offset_, false)) {
    //     memory_system_.AddTransaction(addr_b_ + offset_, false);
    //     inserted_b_ = true;
    // }
    // if (!inserted_c_ &&
    //     memory_system_.WillAcceptTransaction(addr_c_ + offset_, true)) {
    //     memory_system_.AddTransaction(addr_c_ + offset_, true);
    //     inserted_c_ = true;
    // }
    for (int i = 0; i < num_stream; i++) {
        if (!inserted[i] && memory_system_.WillAcceptTransaction(
                                addrs[i] + offset_, (i % 3 == 2))) {
            memory_system_.AddTransaction(addrs[i] + offset_, (i % 3 == 2));
            inserted[i] = true;
        }
    }
    // moving on to next element
    // if (inserted_a_ && inserted_b_ && inserted_c_) {
    //     offset_ += stride_;
    //     inserted_a_ = false;
    //     inserted_b_ = false;
    //     inserted_c_ = false;
    // }
    bool all_inserted = true;
    for (int i = 0; i < num_stream; i++) {
        if (!inserted[i]) {
            all_inserted = false;
            break;
        }
    }
    if (all_inserted) {
        offset_ += stride_;
        for (int i = 0; i < num_stream; i++) {
            inserted[i] = false;
        }
    }
    clk_++;
    return;
}

TraceBasedCPU::TraceBasedCPU(const std::string& config_file,
                             const std::string& output_dir,
                             const std::string& trace_file)
    : CPU(config_file, output_dir) {
    trace_file_.open(trace_file);
    if (trace_file_.fail()) {
        std::cerr << "Trace file does not exist" << std::endl;
        AbruptExit(__FILE__, __LINE__);
    }
}

void TraceBasedCPU::ClockTick() {
    memory_system_.ClockTick();
    while (!trace_file_.eof()) {
        if (get_next_) {
            get_next_ = false;
            trace_file_ >> trans_;
        }
        if (trans_.added_cycle <= clk_) {
            get_next_ = memory_system_.WillAcceptTransaction(trans_.addr,
                                                             trans_.is_write);
            if (get_next_) {
                memory_system_.AddTransaction(trans_.addr, trans_.is_write);
            } else {
                break;
            }
        } else {
            break;
        }
    }
    clk_++;
    return;
}

RamSimCPU::RamSimCPU(const std::string& config_file,
                     const std::string& output_dir,
                     const std::string& trace_file, int cpu_clock_ratio,
                     int mem_clock_ratio)
    : CPU(config_file, output_dir) {
    trace_file_ = new TraceInputStream(trace_file);
    if (!trace_file_->is_open()) {
        std::cerr << "Trace file " << trace_file.c_str() << " failed to open.";
        exit(-1);
    }
    cpu_clock_ratio_ = cpu_clock_ratio;
    mem_clock_ratio_ = mem_clock_ratio;
    printf("cpu_clock_ratio: %d, mem_clock_ratio: %d\n", cpu_clock_ratio_,
           mem_clock_ratio_);
}

void RamSimCPU::ClockTick() {
    memory_system_.ClockTick();
    while (pendReq.size() < maxPendEntry) {
        MemReq record;
        bool eof;
        if (trace_file_->next(&record, &eof)) {
            if (depSolved(&record))
                record.min_issue_cycle =
                    clk_ + ceil(double(record.delay) / cpu_clock_ratio_ *
                                mem_clock_ratio_);
            else
                record.min_issue_cycle = -1;
            if (record.id % 10000 == 0) {
                printf(
                    "id: %ld, addr: %lx, type: %d, delay: %d, size: %d, "
                    "minIssuse: %d, ids: %d\n",
                    record.id, record.addr, record.type, record.delay,
                    record.size, record.min_issue_cycle, record.ids.size());
                printf("clk_: %ld\n", clk_);
            }
            pendReq.push_back(record);
        } else {
            if (!eof) {
                std::cerr << "Trace file has error and has not ended properly."
                          << std::endl;
                exit(-1);
            }
            all_trace_read = true;
            break;
        }
    }

    for (auto it = pendReq.begin(); it != pendReq.end();) {
        if (it->ids.empty()) {
            it = pendReq.erase(it);
            for (auto& req : pendReq) {
                if (req.min_issue_cycle == -1) {
                    if (depSolved(&req)) {
                        req.min_issue_cycle =
                            clk_ + ceil(double(req.delay) / cpu_clock_ratio_ *
                                        mem_clock_ratio_);
                    }
                }
            }
            continue;
        }
        // printf(
        //     "id: %ld, addr: %lx, type: %d, delay: %d, size: %d, "
        //     "minIssuse: %d, ids: %d, dep: %d\n",
        //     it->id, it->addr, it->type, it->delay, it->size,
        //     it->min_issue_cycle, it->ids.size(), it->deps.size());
        if (it->min_issue_cycle >= 0) {
            if (it->min_issue_cycle <= clk_) {
                for (auto id_it = it->ids.begin(); id_it != it->ids.end();) {
                    uint64_t addr = it->addr + *id_it * 64;
                    // addr = addr % ((uint64_t)1 << 10);
                    // addr = 128 + clk_;
                    // if (it->id > 569000)
                    //     printf(
                    //         "addr: %lx, type: %d, WillAcceptTransaction: % "
                    //         "d\n ",
                    //         addr, it->type,
                    //         memory_system_.WillAcceptTransaction(addr,
                    //  it->type));
                    if (memory_system_.WillAcceptTransaction(addr, it->type)) {
                        // printf("addr: %lx, type: %d\n", addr, it->type);
                        memory_system_.AddTransaction(addr, it->type);
                        id_it = it->ids.erase(id_it);
                    } else
                        break;
                }
            }
        }
        ++it;
    }
    clk_++;
    // if (clk_ % 1000 == 0) {
    //     printf("clk: %ld\n", clk_);
    // }
    // if (clk_ > 10000) exit(-1);
    // if (!pendReq.empty()) {
    //     if (pendReq[0].id > 569000) printf("pend id: %ld\n", pendReq[0].id);
    // }
    // printf("pendReq size: %ld\n", pendReq.size());
    if (all_trace_read && pendReq.empty()) finish = true;
}

bool RamSimCPU::finished() { return finish; }

}  // namespace dramsim3
