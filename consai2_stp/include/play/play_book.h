// MIT License
//
// Copyright (c) 2019 ibis-ssl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Created by hans on 2019/09/11.
//

#pragma once

//-------------include----------------//
#include <vector>
#include <memory>
#include <random>
#include <play/test/test_pass.h>
#include <play/entry_point/halt.h>
#include <play/entry_point/stop.h>

#include <play/inplay/ball_in_our_defense.h>

//------------namespace---------------//
//--------------class-----------------//
class PlayBook {
private:
    std::mt19937 mt;
public:
    using RefereeBook = std::tuple<
            HaltPlay,
            StopPlay
    >;

    using InplayBook = std::tuple<
            BallInOurDefensePlay
    >;
    RefereeBook referee_book;
    InplayBook inplay_book;
public:
    PlayBook() {}

    std::unique_ptr<RefereePlay> getPlayByReferee(const GameSituation situation) {
        std::vector<std::unique_ptr<RefereePlay>> available_plays;
        for_each(referee_book, [&](auto play) -> void {
            //itはPlayの子クラス
            std::vector<GameSituation> available_situations = play.getTriggers();
            for (auto s : available_situations) {
                if (s == situation) {
                    //bookの中身をそのままポインタとして外に出すのはまずいので新しく作る．
                    auto new_play_ptr = new decltype(play)();
                    available_plays.emplace_back(new_play_ptr);
                }
            }
        });

        if (available_plays.empty()) {
            return nullptr;
        }

        //ランダムでPlayを選択
        std::uniform_int_distribution<> distribution(0, available_plays.size() - 1);
        return std::move(available_plays.at(distribution(mt)));
    }

    std::unique_ptr<InplayPlay> getPlayOnInplay(const InplaySituation situation) {
        std::vector<std::unique_ptr<InplayPlay>> available_plays;
        for_each(inplay_book, [&](auto play) -> void {
            //itはPlayの子クラス
            std::vector<InplaySituation> available_situations = play.getTriggers();
            for (auto s : available_situations) {
                if (s == situation) {
                    //bookの中身をそのままポインタとして外に出すのはまずいので新しく作る．
                    auto new_play_ptr = new decltype(play)();
                    available_plays.emplace_back(new_play_ptr);
                }
            }
        });

        if (available_plays.empty()) {
            return nullptr;
        }

        //ランダムでPlayを選択
        std::uniform_int_distribution<> distribution(0, available_plays.size() - 1);
        return std::move(available_plays.at(distribution(mt)));
    }

private:
    /**
     * tupleの各要素に対して関数fを実行する
     * @tparam F 関数型
     * @tparam Args タプルの中身の型
     * @param t タプル
     * @param f 関数
     */
    template<typename F, typename ...Args>
    void for_each(std::tuple<Args...> const &t, F f) {
        std::apply([&](auto... args) constexpr { (f(args), ...); },t);
    }
};
