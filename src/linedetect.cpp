#include <list>
#include <vector>
#include <string>
#include <string.h>
#include "rdata.h"
#include "util.h"
enum SectType
{
  // structure
  ST_UNKNOWN, ST_PATCH, ST_LINE, ST_NOISE, ST_COLL,

  // semantic
  ST_WALL_SEEN, ST_WAYOUT, ST_EXTEND, ST_OBJ_INSIDE, ST_OBJ_OUTSIDE, ST_OBJ_ADHERE, ST_DOOR, ST_WALL, ST_OBJECT
};

class RangeSector;
typedef std::list<RangeSector> SectList;
typedef std::list<RangeSector>::iterator SLIter;
typedef std::vector<SLIter> VSLIter;

class RangeSector
{
public:
  RangeSector() :
      sType(ST_UNKNOWN), pType(ST_UNKNOWN)
  {
  }

  SectType sType, pType;

  int iRight, iLeft;

  int sNum;
  float sSpan;
  float sRad;

  Line2d sLine;
  float sDavgN;
  float sDavg;

  VSLIter sSubs;

  int sMark;

  float user_a;
  float user_b;
};

void SplitMerge(RangeData &scan, SectList &result, int i0, int i1)
{
  int rawnum = scan.num;
  char *valid = &scan.valid[0];
  float *range = &scan.range[0];
  float *angle = &scan.angle[0];
  float *posex = &scan.userx[0];
  float *posey = &scan.usery[0];
  SectList &sects = result;

  // algorithm parameters:
  float split_theta = angle[1] - angle[0];
  float split_lamda = _ToRad(4.0f);
  float split_noise = 0.01f;
  float split_distance = 0.05f;
  float line_min_length = 0.32f;

  _Crop(i0, 0, rawnum - 1);
  _Crop(i1, 0, rawnum - 1);
  if (i0 > i1)
    _Swap(i0, i1);

  //// STAGE SPLIT ////
  // step1: split by rupture point
  int lastPos = i0;
  for (int i = i0 + 1; i <= i1 + 1; i++)
  {
    if (i == i1 + 1 || valid[i] != valid[i - 1])
    {
      RangeSector sect;
      sect.sType = valid[i - 1] ? ST_PATCH : ST_UNKNOWN;
      sect.iRight = lastPos;
      sect.iLeft = i - 1;
      sects.push_back(sect);
      lastPos = i;
    }
  }

  // step2: split by breakpoint
  float split_factor = sin(split_theta) / sin(split_lamda - split_theta);
  for (SLIter iter = sects.begin(); iter != sects.end(); iter++)
  {
    if (iter->sType == ST_PATCH)
    {
      int r = iter->iRight;
      int l = iter->iLeft;
      for (int i = r + 1; i <= l; i++)
      {
        float tolerant_distance = range[i - 1] * split_factor + split_noise;
        float current_distance2 = _Square2(posex[i] - posex[i - 1], posey[i] - posey[i - 1]);
        if (current_distance2 > _Square(tolerant_distance))
        {
          sects.insert(iter, *iter);
          iter--;
          iter->iLeft = i - 1;
          iter++;
          iter->iRight = i;
        }
      }
    }
  }

  // step3: split by distant point
  for (SLIter iter = sects.begin(); iter != sects.end(); iter++)
  {
    if (iter->sType == ST_UNKNOWN)
      continue;
    int r = iter->iRight;
    int l = iter->iLeft;
    if (l - r <= 1)
      continue;
    Line2d line(posex[r], posey[r], posex[l], posey[l]);
    float dFarthest = 0.0f;
    int iFarthest = r + 1;
    for (int i = r + 1; i < l; i++)
    {
      float d = _Abs(line.NormalTo(posex[i], posey[i]));
      if (d > dFarthest)
      {
        dFarthest = d;
        iFarthest = i;
      }
    }
    if (dFarthest > split_distance)
    {
      sects.insert(iter, *iter);
      iter->iRight = iFarthest + 1;
      iter--;
      iter->iLeft = iFarthest;
      if (iter != sects.begin())
        iter--;
    }
  }

  //// STAGE MERGE ////

  // for sectors with sType == ST_PATCH
  // all points laid in a strip with width < 2 * splite_distance
  for (SLIter iter = sects.begin(); iter != sects.end(); iter++)
  {
    int r = iter->iRight;
    int l = iter->iLeft;
    iter->sNum = l - r + 1;
    iter->sRad = split_theta * iter->sNum;
    if (iter->sType == ST_UNKNOWN)
      continue;
    iter->sSpan = _Distance2(posex[r], posey[r], posex[l], posey[l]);

    if (iter->sSpan >= line_min_length)
    {
      iter->sLine.BuildLine(posex + r, posey + r, iter->sNum);
      iter->sType = ST_LINE;
      iter->sMark = 1;
    }
  }

  // step1: merge neighboring lines
  // step2: merge lines not neighboring

  for (int step = 1; step <= 2; step++)
  {
    SLIter chosen = sects.end();
    int mark_valid = 2 - step;
    while (1)
    {
      SLIter _end = sects.end();
      if (chosen == _end)
      {
        // find longest merge-able line <chosen>
        for (SectList::iterator iter = sects.begin(); iter != _end; iter++)
        {
          if (iter->sType == ST_LINE && iter->sMark == mark_valid)
          {
            if (chosen == _end || iter->sLine.length > chosen->sLine.length)
              chosen = iter;
          }
        }
        if (chosen == _end)
          break;
      }

      // try merge with its neighbor
      bool merged = false;
      SLIter iter_r, iter_l;
      if (step == 1)
      {
        iter_l = iter_r = chosen;
        if (iter_r != sects.begin())
          iter_r--;
        iter_l++;
        if (iter_l != _end)
          iter_l++;
      }
      else
      {
        iter_r = sects.begin();
        iter_l = _end;
      }

      for (SLIter iter = iter_r; iter != iter_l; iter++)
      {
        if (iter != chosen && iter->sType == ST_LINE && iter->sMark == mark_valid
            && chosen->sLine.Collineation(iter->sLine, step == 1 ? 0.125f : 0.225f) == 1)
        {
          if (step == 1)
          {
            if (iter == iter_r)
              chosen->iRight = iter->iRight;
            else
              chosen->iLeft = iter->iLeft;
            int r = chosen->iRight;
            int l = chosen->iLeft;
            chosen->sNum = l - r + 1;
            chosen->sRad = split_factor * chosen->sNum;
            chosen->sSpan = _Distance2(posex[l], posey[l], posex[r], posey[l]);
            chosen->sLine.BuildLine(posex + r, posey + r, chosen->sNum);
            sects.erase(iter);
          }
          else
          {
            if (chosen->sType == ST_LINE)
            {
              // new cluster
              SectList::iterator iter_new, iter_1, iter_2;
              if (chosen->iRight < iter->iRight)
              {
                iter_1 = chosen;
                iter_2 = iter;
              }
              else
              {
                iter_1 = iter;
                iter_2 = chosen;
              }
              sects.insert(iter_1, *iter_1);
              iter_new = iter_1;
              iter_new--;
              int r1 = iter_1->iRight;
              int r2 = iter_2->iRight;
              int l2 = iter_2->iLeft;
              iter_new->sType = ST_COLL;
              iter_new->iLeft = l2;
              iter_new->sNum = iter_1->sNum + iter_2->sNum;
              iter_new->sRad = (l2 - r1 + 1) * split_theta;
              iter_new->sSpan = _Distance2(posex[r1], posey[r1], posex[l2], posey[l2]);
              iter_new->sSubs.push_back(iter_1);
              iter_new->sSubs.push_back(iter_2);
              vec_f4 vec_x(iter_new->sNum);
              vec_f4 vec_y(iter_new->sNum);
              memcpy(&vec_x[0], &posex[r1], sizeof(float) * iter_1->sNum);
              memcpy(&vec_x[iter_1->sNum], &posex[r2], sizeof(float) * iter_2->sNum);
              memcpy(&vec_y[0], &posey[r1], sizeof(float) * iter_1->sNum);
              memcpy(&vec_y[iter_1->sNum], &posey[r2], sizeof(float) * iter_2->sNum);
              iter_new->sLine.BuildLine(&vec_x[0], &vec_y[0], iter_new->sNum);
              iter_new->sMark = mark_valid;
              iter_1->sSubs.push_back(iter_new);
              iter_1->sMark = 1 - mark_valid;
              iter_2->sSubs.push_back(iter_new);
              iter_2->sMark = 1 - mark_valid;
              chosen = iter_new;
            }
            else
            {
              // clustering
              int pos;
              if (chosen->iRight > iter->iLeft)
                pos = 0; // on right
              else if (chosen->iLeft < iter->iRight)
                pos = 2; // on left
              else
                // chosen->iRight < iter->iRight < iter->iLeft < chosen->iLeft
                pos = 1; // in between

              if (pos != 1)
              {
                if (pos == 0)
                  chosen->iRight = iter->iRight;
                else
                  chosen->iLeft = iter->iLeft;

                chosen->sRad = (chosen->iLeft - chosen->iRight + 1) * split_theta;
                chosen->sSpan = _Distance2(posex[chosen->iRight], posey[chosen->iRight], posex[chosen->iLeft], posey[chosen->iLeft]);
              }
              chosen->sNum += iter->sNum;
              chosen->sSubs.push_back(iter);
              iter->sSubs.push_back(chosen);
              for (int i = (int)chosen->sSubs.size() - 1; i >= 1; i--)
                if (chosen->sSubs[i]->iRight < chosen->sSubs[i - 1]->iRight)
                  _Swap(chosen->sSubs[i], chosen->sSubs[i - 1]);

              vec_f4 vec_x(chosen->sNum);
              vec_f4 vec_y(chosen->sNum);
              for (int i = 0, anchor = 0; i < (int)chosen->sSubs.size(); i++)
              {
                SLIter sec = chosen->sSubs[i];
                memcpy(&vec_x[anchor], &posex[sec->iRight], sizeof(float) * sec->sNum);
                memcpy(&vec_y[anchor], &posey[sec->iRight], sizeof(float) * sec->sNum);
                anchor += sec->sNum;
              }
              chosen->sLine.BuildLine(&vec_x[0], &vec_y[0], chosen->sNum);
              iter->sMark = 1 - mark_valid;

              if (pos == 0)
              {
                sects.insert(iter, *chosen);
                sects.erase(chosen);
                chosen = iter;
                chosen--;
                for (int i = 0; i < (int)chosen->sSubs.size(); i++)
                  chosen->sSubs[i]->sSubs[0] = chosen;
              }
            }
          }
          merged = true;
          break;
        } //end if (iter != chosen &&
      } // end for(SectList::iterator iter = iter_r;

      // if neither neighbor can be merged with then set it to not merge-able
      if (!merged)
      {
        chosen->sMark = 1 - mark_valid;
        chosen = _end;
      }
    }
  }

  for (SLIter iter = sects.begin(); iter != sects.end(); iter++)
  {
    if (iter->sType == ST_LINE)
    {
      int r = iter->iRight;
      int l = iter->iLeft;
      Line2d &line = iter->sLine;
      float Ed = 0;
      float Edn = 0;
      int dnum = 0;
      const int N = 8;
      float maxd[N + 1];
      for (int i = r; i <= l; i++)
      {
        float d = _Abs(line.NormalTo(posex[i], posey[i]));
        Ed += d;
        maxd[dnum] = d;
        for (int j = dnum; j > 0; j--)
        {
          if (maxd[j] > maxd[j - 1])
            _Swap(maxd[j], maxd[j - 1]);
          else
            break;
        }
        if (dnum < N)
          dnum++;
      }
      for (int i = 0; i < N; i++)
        Edn += maxd[i];
      iter->sDavg = Ed / iter->sNum;
      iter->sDavgN = Edn / N;
    }
  }
}
