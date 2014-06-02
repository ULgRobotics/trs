function a = qdot(q0, q1)
% qdot Dot product of two quaternions.
%   qdot returns the dot product of two elements of the Quaternion class
%   from P. Corke's RVT.

% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

  a = [q0.s q0.v] * [q1.s q1.v]';

end
