/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include <mutex>

namespace ORB_SLAM3
{

    class KeyFrame;
    class Frame;
    class Map;

    class KeyFrameDatabase
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar & mvBackupInvertedFileId;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KeyFrameDatabase() {}
        KeyFrameDatabase(const ORBVocabulary &voc);

        void add(KeyFrame *pKF);

        void erase(KeyFrame *pKF);

        void clear();
        void clearMap(Map *pMap);

        // Loop Detection(DEPRECATED)
        std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

        // Loop and Merge Detection
        void DetectCandidates(KeyFrame *pKF, float minScore, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand);
        void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nMinWords);
        void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame *> &vpLoopCand, vector<KeyFrame *> &vpMergeCand, int nNumCandidates);

        // Relocalization
        std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F, Map *pMap);

        void PreSave();
        void PostLoad(map<long unsigned int, KeyFrame *> mpKFid);
        void SetORBVocabulary(ORBVocabulary *pORBVoc);

    protected:
        // Associated vocabulary
        const ORBVocabulary *mpVoc;

        // Inverted file
        std::vector<list<KeyFrame *>> mvInvertedFile;

        // For save relation without pointer, this is necessary for save/load function
        std::vector<list<long unsigned int>> mvBackupInvertedFileId;

        // Mutex
        std::mutex mMutex;
    };

} // namespace ORB_SLAM

#endif
