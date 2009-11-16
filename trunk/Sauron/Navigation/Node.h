#ifndef __NAVIGATION_NODE_H__
#define __NAVIGATION_NODE_H__

#include <vector>
#include "BaseTypes.h"
#include "CustomTypes.h"
#include "Point2D.h"


namespace sauron
{

class Node
{
    public:
        enum NodeType
        {
            PRIMARY,
            SECONDARY,
            TEMPORARY,
        };

        Node();
        Node( const Point2D<pose_t> &position );
        Node( const Point2D<pose_t> &position, const NodeType &type );
		Node( const Point2D<pose_t> &position, const NodeType &type, const std::string& name );
        ~Node();

		std::string getName() const { return m_name; }

        void setPosition( const Point2D<pose_t> &position );
        const Point2D<pose_t> &getPosition() const;

        void addAdjacent( Node &other );
        const std::vector<Node *> &getAdjacents() const;

        bool operator <  ( const Node &other ) const;
        bool operator == ( const Node &other ) const;
        bool operator != ( const Node &other ) const;

        NodeType &Type();
        const NodeType &Type() const;

   private:
		std::string m_name;
        Point2D<pose_t>     m_position;
        std::vector<Node *> m_adjacents;
        NodeType            m_type;
};


typedef Node Waypoint;
typedef std::vector<Node> Graph;

}   // namespace sauron

#endif  // __NAVIGATION_NODE_H__