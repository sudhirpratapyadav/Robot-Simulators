import inspect
import os
import xml.etree.ElementTree as ET
import igraph
import plotly.graph_objects as go

def parse_urdf(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    return root

def build_tree(robot, graph, parent=None):
    for link in robot.findall(".//link"):
        link_name = link.attrib["name"]
        graph.add_vertex(link_name)
        if parent is not None:
            graph.add_edge(parent, link_name)

    for joint in robot.findall(".//joint"):
        parent_element = joint.find("./parent")
        child_element = joint.find("./child")

        if parent_element is not None and child_element is not None:
            parent_link = parent_element.attrib["link"]
            child_link = child_element.attrib["link"]
            joint_name = joint.attrib["name"]
            graph.add_edge(parent_link, child_link, joint=joint_name)

def display_tree(graph):
    lay = graph.layout_reingold_tilford(root=[0])  # Reingold-Tilford layout for tree structure

    position = {k: lay[k] for k in range(len(lay))}
    Xn = [position[k][0] for k in range(len(lay))]
    Yn = [position[k][1] for k in range(len(lay))]

    labels = graph.vs["name"]
    joint_labels = graph.es["joint"]

    edge_x = []
    edge_y = []
    edge_x_txt = []
    edge_y_txt = []
    edge_text = []
    for edge in graph.get_edgelist():
        x0, y0 = position[edge[0]]
        x1, y1 = position[edge[1]]
        edge_x += [x0, x1, None]
        edge_y += [y0, y1, None]

        edge_x_txt.append((x0 + x1) / 2)  # middle point x-coordinate
        edge_y_txt.append((y0 + y1) / 2)  # middle point y-coordinate
        edge_text.append(joint_labels[graph.get_eid(edge[0], edge[1])])
    
    edge_trace = go.Scatter(
        x=edge_x, y=edge_y,
        line=dict(width=0.5, color='#888'),
        mode='lines'
        )
    

    node_x = []
    node_y = []
    for x, y in position.values():
        node_x.append(x)
        node_y.append(y)

    node_trace = go.Scatter(
        x=node_x, y=node_y,
        mode='markers',
        hoverinfo='text',
        marker=dict(
            size=10,
        )
        # marker=dict(
        #     showscale=True,
        #     colorscale='YlGnBu',
        #     size=10,
        #     # colorbar=dict(
        #     #     thickness=15,
        #     #     title='Node Connections',
        #     #     xanchor='left',
        #     #     titleside='right'
        #     # )
        # )
    )

    # Add annotations with names
    annotations = [
        dict(
            text=labels[k],
            x=node_x[k], y=node_y[k],
            # xref='x1', yref='y1',
            font=dict(color='rgb(50,50,50)', size=10),
            showarrow=False
            )
        for k in range(len(labels))]

    annotations+= [
        dict(
            text=edge_text[k],
            x=edge_x_txt[k], y=edge_y_txt[k],
            # xref='x1', yref='y1',
            font=dict(color='rgb(50,50,50)', size=10),
            showarrow=False
            )
        for k in range(len(edge_text))]

    fig = go.Figure(
        data=[edge_trace, node_trace],
        layout=go.Layout(
            showlegend=False,
            hovermode='closest',
            margin=dict(b=0, l=0, r=0, t=0),
            xaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            yaxis=dict(showgrid=False, zeroline=False, showticklabels=False),
            annotations=annotations
            ))

    fig.show()

if __name__ == "__main__":
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    urdf_file_path = os.path.join(currentdir, "./interbotix_descriptions/urdf/wx250s.urdf")
    robot = parse_urdf(urdf_file_path)

    # Create a directed graph using igraph
    robot_graph = igraph.Graph(directed=True)

    # Build the tree structure
    build_tree(robot, robot_graph)

    # Display the tree structure
    display_tree(robot_graph)