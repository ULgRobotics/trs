function a = qminus(q)
% qminus Additive inverse of the four coordinates of a quaternion.
%   qminus returns a quaternion whose elements are the additive inverse of the
%   elements of q. The variable q must be of class Quaternion from P. Corke's
%   RVT. If q is a unit quaternion, qminus(q) represents the same rotation as
%   q, but lies at the antipodal point on the 3-sphere.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  a = Quaternion(-[q.s q.v]);

end
