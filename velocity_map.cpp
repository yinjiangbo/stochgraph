#include "velocity_map.hpp"


void ReadInterval(const xmlpp::Node* node, string attribute_name);
void ReadEdge(const xmlpp::Node* node, string attribute_name);
float ReadLane(const xmlpp::Node* node, string attribute_name);

float VelocityMap::GetVelocity(const stoch_edge* edge) const{
   if (velocities_.find(edge->get_sumo_id()) == velocities_.end()) {
      return edge->get_speedlimit();
   } else {
      return velocities_.find(edge->get_sumo_id())->second;
   }
}

void VelocityMap::ReadVelocities(string filename) {
      xmlpp::DomParser parser;
      parser.parse_file(filename);

      //Walk the tree:
      const xmlpp::Node* pNode = parser.get_document()->get_root_node();

      xmlpp::Node::NodeList list = pNode->get_children();
      for(xmlpp::Node::NodeList::iterator iter = list.begin();
          iter != list.end();
          ++iter)
      {
         ReadInterval(*iter, "speed");
      }
}

void VelocityMap::ReadInterval(const xmlpp::Node* node, string attribute_name) {
   const Glib::ustring nodename = node->get_name();

   // Ignore node elements.
   if (nodename == "interval") {
      const xmlpp::Element* nodeElement =
         dynamic_cast<const xmlpp::Element*>(node);

      if(nodeElement) {
         // Gets the attributes
         const xmlpp::Element::AttributeList& attributes =
            nodeElement->get_attributes();

         for(auto iter = attributes.begin();
             iter != attributes.end();
             ++iter) {

            const xmlpp::Attribute* attribute = *iter;
            const Glib::ustring namespace_prefix =
               attribute->get_namespace_prefix();

            if (attribute->get_name() == "begin") {
               float begin_time = atof(attribute->get_value().c_str());
            }
            if (attribute->get_name() == "end") {
               float end_time = atof(attribute->get_value().c_str());
            }
         }

         // Finished with this element.

         // Reads the children.
         xmlpp::Node::NodeList list = node->get_children();
         for(xmlpp::Node::NodeList::iterator iter = list.begin();
             iter != list.end();
             ++iter)
         {
            ReadEdge(*iter, attribute_name);
         }
      }
   }
}

void VelocityMap::ReadEdge(const xmlpp::Node* node, string attribute_name) {
   const Glib::ustring nodename = node->get_name();

   // Ignore node elements.
   if (nodename == "edge") {
      const xmlpp::Element* nodeElement =
         dynamic_cast<const xmlpp::Element*>(node);

      if(nodeElement) {

         string id;

         // Gets the attributes
         const xmlpp::Element::AttributeList& attributes =
            nodeElement->get_attributes();

         for(auto iter = attributes.begin();
             iter != attributes.end();
             ++iter) {

            const xmlpp::Attribute* attribute = *iter;
            const Glib::ustring namespace_prefix =
               attribute->get_namespace_prefix();

            if (attribute->get_name() == "id") {
               id = attribute->get_value();
            }
         }

         // Finished with this element.
         float min_value = std::numeric_limits<float>::max();
         int size = 0;

         // Reads the children.
         xmlpp::Node::NodeList list = node->get_children();
         for(xmlpp::Node::NodeList::iterator iter = list.begin();
             iter != list.end();
             ++iter) {
            float value = ReadLane(*iter, attribute_name);
            if (value > -1) {
               min_value = std::min(value, min_value);
               size++;
            }
         }

         if (attribute_name == "speed") {
            velocities_[id] = min_value;
         }
      }
   }
}

float VelocityMap::ReadLane(const xmlpp::Node* node, string attribute_name) {
   const Glib::ustring nodename = node->get_name();

   // Ignore node elements.
   if (nodename == "lane") {
      const xmlpp::Element* nodeElement =
         dynamic_cast<const xmlpp::Element*>(node);

      float value;

      if(nodeElement) {

         // Gets the attributes
         const xmlpp::Element::AttributeList& attributes =
            nodeElement->get_attributes();

         for(auto iter = attributes.begin();
             iter != attributes.end();
             ++iter) {

            const xmlpp::Attribute* attribute = *iter;
            const Glib::ustring namespace_prefix =
               attribute->get_namespace_prefix();

            if (attribute->get_name() == attribute_name) {
               value = atof(attribute->get_value().c_str());
            }
         }

         // Finished with this element.
         return value;
      }
      else
         return -1;
   }
   else
      return -1;
}
