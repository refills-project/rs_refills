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
          << "owl_has(P, 'http://www.w3.org/2000/01/rdf-schema#subClassOf',Z),"
          << "owl_restriction(Z,restriction(shop:'articleNumberOfProduct',has_value(AN))),"
          << "rdf_has(AN, shop:dan, literal(type(_,DAN))),"
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
  plQuery<< "owl_has('"<<facing.productId<<"', 'http://www.w3.org/2000/01/rdf-schema#subClassOf',Zd),"
         << "owl_restriction(Zd,restriction(shop:'depthOfProduct',has_value(literal(type(_,D_XSD))))),atom_number(D_XSD,D),"
         << "owl_has('"<<facing.productId<<"', 'http://www.w3.org/2000/01/rdf-schema#subClassOf',Zw),"
         << "owl_restriction(Zw,restriction(shop:'widthOfProduct',has_value(literal(type(_,W_XSD))))),atom_number(W_XSD,W),"
         << "owl_has('"<<facing.productId<<"', 'http://www.w3.org/2000/01/rdf-schema#subClassOf',Zh),"
         << "owl_restriction(Zh,restriction(shop:'heightOfProduct',has_value(literal(type(_,H_XSD))))),atom_number(H_XSD,H).";

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

   plQuery << "rdf_has('" << facing_id << "', shop:leftSeparator, L), object_feature(L, LF, dmshop:'DMShelfPerceptionFeature'),object_frame_name(LF,LFrameName),"
           << "rdf_has('" << facing_id << "', shop:rightSeparator,R), object_feature(R, RF, dmshop:'DMShelfPerceptionFeature'),object_frame_name(RF,RFrameName).";
   
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
   plQuery << "rdf_has('" << facing_id
           << "', shop:mountingBarOfFacing, M), object_perception_affordance_frame_name(M,MFrameName).";
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
