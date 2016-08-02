/*
 * Copyright (c) 2012-2013 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of a Recap tag store.
 */

#include "debug/CacheRepl.hh"
#include "mem/cache/tags/recap/recap.hh"
#include "mem/cache/base.hh"

Recap::Recap(const Params *p)
    : BaseSetAssoc(p)
{
}

void
Recap::regStats()
{
    using namespace Stats;
    BaseTags::regStats();

    block_req
        .name(name() + ".block_req")
        .desc("The number of ways for each core to achieve it's highest utilisation.")
        .flags(total | nozero | nonan)
        ;

    missCounter
        .init(assoc)
        .name(name() + ".missCounter")
        .desc("Calculate the sum of the misses.")
        ;
}

CacheBlk*
Recap::accessBlock(ThreadID threadId, Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    CacheBlk *blk = BaseSetAssoc::accessBlock(threadId, addr, is_secure, lat, master_id);

    if (blk != nullptr) {//Hit
        // move this block to head of the MRU list
       for(int sd = 0; sd < assoc; sd++){//sd:stack distance

           if (sets[blk->set].blks[sd] == blk){

                if (blk->way < allocAssoc){//Hit

                   numMissesCounter[sd]++;
                }
           }
       }
        sets[blk->set].moveToHead(blk);
        DPRINTF(CacheRepl, "set %x: moving blk %x (%s) to MRU\n",
                blk->set, regenerateBlkAddr(blk->tag, blk->set),
                is_secure ? "s" : "ns");
    }
    else {//Miss
        numMissesCounter[assoc]++;
    }

    block_req = getMaxMuWays(threadId);

    for (int a = 0; a < assoc; a++){

        missCounter[a] = getNumMisses(a);
     }
    return blk;
}

CacheBlk*
Recap::findVictim(Addr addr)
{
    int set = extractSet(addr);
    // grab a replacement candidate
    BlkType *blk = nullptr;
    for (int i = assoc - 1; i >= 0; i--) {
        BlkType *b = sets[set].blks[i];
        if (b->way < allocAssoc) {
            blk = b;
            break;
        }
    }
    assert(!blk || blk->way < allocAssoc);

    if (blk && blk->isValid()) {
        DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n",
                set, regenerateBlkAddr(blk->tag, set));
    }

    return blk;
}

void
Recap::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);

    int set = extractSet(pkt->getAddr());
    sets[set].moveToHead(blk);
}

void
Recap::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);

    // should be evicted before valid blocks
    int set = blk->set;
    sets[set].moveToTail(blk);
}

Recap*
RecapParams::create()
{
    return new Recap(this);
}

/**
* Calculate the number of misses according to the different associativity.
* @param numWays:the associativity (must no more than l2cache associativity)
* @return        the number of misses in this associativity
*/
int
Recap::getNumMisses(int num_ways){

    int numMisses = numMissesCounter[assoc];//Miss

	for (int i = num_ways; i < assoc; i++){//Hit
		numMisses = numMissesCounter[i] + numMisses;
	}

	return numMisses;
}

/**
 * Find the minimum blocks to get highest utilisation(max_mu).
 * @param core_id: the number of the core
 * @return         the minimum blocks
 */
int
Recap::getMaxMuWays(int core_id){
	double max_mu = 0;
	int max_mu_ways = 1;

	for (int j = 1; j <= assoc; j++){

		double utility = getNumMisses(0) - getNumMisses(j);
		double mu = utility / j;

		if (mu > max_mu){
			max_mu = mu;
			max_mu_ways = j;
		}
	}
	return max_mu_ways;
}