#pragma once

#include <rosprolog/rosprolog_client/PrologClient.h>
#include <rs_refills/facing.h>

inline bool is_layer_hanging(const std::string facing_id)
{
  std::stringstream plQuery;
  PrologClient pl;
  plQuery << "shelf_facing(F,'" << facing_id << "'),shelf_layer_standing(F).";
  outInfo("Asking query: " << plQuery.str());
  PrologQuery bdgs = pl.query(plQuery.str());

  if (bdgs.begin() == bdgs.end())
    return true;
  else
    return false;
}


inline bool get_facing_dimensions(const std::string &facing_id, Facing &facing)
{
  std::stringstream plQuery;
  PrologClient pl;
  plQuery << "shelf_facing_product_type('" << facing.facingId << "', P),"
          << "subclass_of(P, Z),"
          << "has_description(Z,value(shop:'articleNumberOfProduct', AN)),"
          << "triple(AN, shop:dan, DAN),"
          << "comp_facingWidth('" << facing.facingId << "', W),"
          << "comp_facingHeight('" << facing.facingId << "',H).";
   outInfo("Asking query: " << plQuery.str());
   PrologQuery bdgs = pl.query(plQuery.str());
   if (bdgs.begin() == bdgs.end())
   {
      return false;
   }
   for (auto bdg : bdgs)
   {
       facing.width = bdg["W"];
       facing.height = bdg["H"];
       facing.productId = bdg["P"].toString();
       facing.gtin = bdg["AN"].toString();
       facing.dan = bdg["DAN"].toString();
       size_t loc = facing.gtin.find_last_of("GTIN_");
       loc != std::string::npos ? facing.gtin = facing.gtin.substr(loc+1, facing.gtin.size() - loc-2) : facing.gtin = "";
   }

   return true;
}

inline bool get_product_dim_for_facing(Facing &facing)
{

  std::stringstream plQuery;
  PrologClient pl;
  plQuery << "subclass_of('" << facing.productId << "', Zd),"
          << "has_description(Zd,value(shop:'depthOfProduct', D)),"
          << "subclass_of('" << facing.productId << "', Zw),"
          << "has_description(Zw,value(shop:'widthOfProduct', W)),"
          << "subclass_of('" << facing.productId << "', Zh),"
          << "has_description(Zh,value(shop:'heightOfProduct', H)).";

  outInfo("Asking query: " << plQuery.str());
  PrologQuery bdgs = pl.query(plQuery.str());
  if(bdgs.begin() == bdgs.end()) {
     outWarn("No solution to query: " << plQuery.str());
     facing.productDims.d = 0.41;
     facing.productDims.h = facing.height;
     facing.productDims.w = facing.width;
  }
  for(auto bdg : bdgs) {
     facing.productDims.d = bdg["D"];
     facing.productDims.h = bdg["H"];
     facing.productDims.w = bdg["W"];
     break;                                                               }
}


//returns false if there are no separators
inline bool get_separator_frame_ids_for_facing(const std::string &facing_id, std::string &leftSepTFId, 
                                           std::string &rightSepTFId)
{
   std::stringstream plQuery;
   PrologClient pl;

   plQuery << "triple('" << facing_id << "', shop:leftSeparator, L), object_feature_type(L, LF, dmshop:'DMShelfPerceptionFeature'),holds(LF,knowrob:frameName, LFrameName),"
           << "triple('" << facing_id << "', shop:rightSeparator,R), object_feature_type(R, RF, dmshop:'DMShelfPerceptionFeature'),holds(RF, knowrob:frameName, RFrameName).";

   outInfo("Asking query: " << plQuery.str());
   PrologQuery bdgs = pl.query(plQuery.str());
   if (bdgs.begin() == bdgs.end())
   {
     outError("No results found the left and right separator. are you sure this is the right facing type?");
     return false;
   }
   for (auto bdg : bdgs)
   {
     leftSepTFId = bdg["LFrameName"].toString();
     rightSepTFId = bdg["RFrameName"].toString();
     break;
   }
   return true;
}

inline bool get_mounting_bar_frame_id_for_facing(const std::string &facing_id, std::string &frame_id)
{
   std::stringstream plQuery;
   PrologClient pl;
   plQuery << "triple('" << facing_id
           << "', shop:mountingBarOfFacing, M), triple(Obj, knowrob:hasAffordance, Affordance), instance_of(Affordance,"
           << "knowrob:PerceptionAffordance), holds(Affordance, knowrob:frameName, AffFrameName).";
   outInfo("Asking query: " << plQuery.str());
   PrologQuery bdgs = pl.query(plQuery.str());
   if (bdgs.begin() == bdgs.end())
   {
     return false;
   }
   for (auto bdg : bdgs)
   {
      frame_id = bdg["MFrameName"].toString();
      frame_id = frame_id.substr(1, frame_id.size() - 2);
      break;
   }
   return true;

}