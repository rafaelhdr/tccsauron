#include "Node.h"
#include "MathHelper.h"

namespace sauron
{

Node::Node()
    : m_type( PRIMARY )
{
}


Node::Node( const Point2D<pose_t> &position )
    : m_position( position ),
      m_type( PRIMARY )
{
}


Node::Node( const Point2D<pose_t> &position, const NodeType &type )
    : m_position( position ),
      m_type( type )
{
}


Node::~Node()
{
}


void Node::setPosition( const Point2D<pose_t> &position )
{
    m_position = position;
}


const Point2D<pose_t> &Node::getPosition() const
{
    return m_position;
}


void Node::addAdjacent( Node &other )
{
    m_adjacents.push_back( &other );
}


const std::vector<Node *> &Node::getAdjacents() const
{
    return m_adjacents;
}


bool Node::operator < ( const Node &other ) const
{
    if ( m_position.X() * m_position.Y() < other.m_position.X() * other.m_position.Y() )
        return true;
    
    return false;
}


bool Node::operator == ( const Node &other ) const
{
    if ( floating_point::isEqual( m_position.X(), other.m_position.X() ) &&
         floating_point::isEqual( m_position.Y(), other.m_position.Y() ) )
        return true;

    return false;
}


bool Node::operator != ( const Node &other ) const
{
    if ( floating_point::isEqual( m_position.X(), other.m_position.X() ) &&
         floating_point::isEqual( m_position.Y(), other.m_position.Y() ) )
        return false;

    return true;
}

Node::NodeType &Node::Type()
{
    return m_type;
}

const Node::NodeType &Node::Type() const
{
    return m_type;
}


}   // namespace sauron