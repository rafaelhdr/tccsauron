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
            SECUNDARY,
            TEMPORARY,
        };

        Node();
        Node( const Point2D<pose_t> &position );
        Node( const Point2D<pose_t> &position, const NodeType &type );
        ~Node();

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
        Point2D<pose_t>     m_position;
        std::vector<Node *> m_adjacents;
        NodeType            m_type;
};


typedef Node Waypoint;
typedef std::vector<Node> Graph;

}   // namespace sauron

#endif  // __NAVIGATION_NODE_H__